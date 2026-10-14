import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
import importlib.util
from pathlib import Path
import socket
import threading
import time
import unittest
spec = importlib.util.spec_from_file_location('cubli_ui', Path(__file__).resolve().parents[1]/'Software/main.py')
ui = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ui)
app = ui.QApplication.instance() or ui.QApplication([])
STATE = '$STATE,App:0,Dbg:2,Mode:0,BMode:1,Bal:0,Cal:1,IMU:1,Aligned:111,EN:010,Fault:0,Trip:0,Tilt:0.12,U1:0.00,U2:0.25,U3:0.00#'
AUTO_STATE = STATE.replace('App:0','App:1').replace('EN:010','EN:000').replace('Aligned:111','Aligned:100').replace('#',
    ',Auto:0,BReason:0,FullSpeed:250.0,FullMs:0,RetryMs:0,EdgeReady:1,Wheel:0.0,Rate:0.0,Angle:0.1,PoseTilt:0.1,TripDeg:20.0,VLimit:6.0,SLimit:330.0,Ref:1,AxesOK:1,DirOK:1,Axis1:2,D1:-1#')
class UpperTests(unittest.TestCase):
    def setUp(self):
        self.win = ui.MainWindow()
        self.win.worker.is_running = True
        self.win.worker._process_received_line(STATE)
    def tearDown(self):
        self.win.worker.is_running = False
        self.win.close()
    def queued(self):
        return [item[0] for item in self.win.worker.tx_queue]
    def test_three_axis_controls(self):
        self.assertTrue(self.win.card_m1.spin_speed.isEnabled())
        self.assertTrue(self.win.card_m2.spin_speed.isEnabled())
        self.assertTrue(self.win.card_m3.spin_speed.isEnabled())
        self.assertTrue(self.win.card_m2.chk_enable.isChecked())
        self.assertFalse(self.win.sync_group.isEnabled())
    def test_independent_unlock_and_batch_changed_axes(self):
        self.win.worker._process_received_line(STATE.replace('Aligned:111','Aligned:101'))
        self.assertTrue(self.win.card_m1.spin_speed.isEnabled())
        self.assertFalse(self.win.card_m2.spin_speed.isEnabled())
        self.assertTrue(self.win.card_m3.spin_speed.isEnabled())
        self.win.on_motor_speed_change(1, 20)
        self.win.on_motor_speed_change(3, -30)
        self.win._flush_speed_command()
        self.assertEqual(self.queued(), ['M1 20.00','M3 -30.00'])
        self.assertEqual(self.win.debounce_timer.interval(),20)
        self.assertEqual(self.win.plot_timer.interval(),100)
    def test_reference_required_and_mapping_visible(self):
        state=STATE.replace('App:0','App:1').replace('#', ',Ref:0,Map1:3,Map2:1,Map3:2,PP1:7,PP2:14,PP3:7,Dir1:1,Dir2:-1,Dir3:1#')
        self.win.worker._process_received_line(state)
        self.assertFalse(self.win.btn_balance_start.isEnabled())
        self.assertIn('ENC1', self.win.card_m2.title())
        self.assertIn('PP14', self.win.card_m2.title())
        self.win.worker._process_received_line(state.replace('Ref:0','Ref:1'))
        self.assertTrue(self.win.btn_balance_start.isEnabled())
    def test_auto_edge_start_stop_and_protection_status(self):
        self.win.worker._process_received_line(AUTO_STATE)
        self.assertTrue(self.win.btn_auto_edge.isEnabled())
        self.win.btn_auto_edge.click()
        self.assertEqual(self.queued(),['BAUTO 1'])
        waiting=AUTO_STATE.replace('Auto:0','Auto:1').replace('Bal:0','Bal:6').replace('BReason:0','BReason:2').replace('RetryMs:0','RetryMs:500')
        self.win.worker._process_received_line(waiting)
        self.assertIn('满速持续3秒',self.win.lbl_edge_detect.text())
        self.assertIn('0.5秒',self.win.lbl_edge_detect.text())
        self.assertTrue(self.win.btn_auto_edge.isEnabled())
        self.win.btn_auto_edge.click()
        self.assertEqual(self.queued(),['BAUTO 0'])
    def test_auto_edge_requires_reference_limit_and_selected_axis(self):
        for bad in (AUTO_STATE.replace('Ref:1','Ref:0'),AUTO_STATE.replace('Aligned:100','Aligned:000'),
                    AUTO_STATE.replace('SLimit:330.0','SLimit:180.0'),AUTO_STATE.replace('BMode:1','BMode:4')):
            self.win.worker._process_received_line(bad)
            self.assertFalse(self.win.btn_auto_edge.isEnabled())
        self.win.worker._process_received_line(STATE)
        self.assertFalse(self.win.btn_auto_edge.isEnabled()) # Older firmware lacks Auto.
    def test_edge_profile_waits_for_ack_and_fresh_matching_state(self):
        self.win.worker._process_received_line(AUTO_STATE)
        self.win.btn_edge_profile.click()
        self.assertTrue(self.win.edge_profile_pending)
        self.assertIn('LQR1 80 5 -0.045 0',self.queued())
        self.assertIn('BFULL 250',self.queued())
        self.assertNotIn('BAUTO 1',self.queued())
        self.win.worker._process_received_line(AUTO_STATE)
        self.assertTrue(self.win.edge_profile_pending)
        self.assertFalse(self.win.btn_auto_edge.isEnabled())
        self.win.worker._process_received_line('OK BREF')
        self.win.worker._process_received_line(AUTO_STATE)
        self.assertFalse(self.win.edge_profile_pending)
        self.assertTrue(self.win.btn_auto_edge.isEnabled())
        self.assertAlmostEqual(self.win.balance_spins[2].value(),-.045)
    def test_edge_profile_error_cancels_remaining_commands(self):
        self.win.worker._process_received_line(AUTO_STATE)
        self.win.btn_edge_profile.click()
        self.win.worker._process_received_line('ERR BREF')
        self.assertFalse(self.win.edge_profile_pending)
        self.assertEqual(self.queued(),['STOP'])
    def test_auto_fields_validation_and_user_edit_preservation(self):
        self.win.worker._process_received_line(AUTO_STATE)
        self.win.spin_full_speed.setValue(270)
        self.win.spin_trip_angle.setValue(25)
        self.win.worker._process_received_line(AUTO_STATE)
        self.assertEqual(self.win.spin_full_speed.value(),270)
        self.assertEqual(self.win.spin_trip_angle.value(),25)
        before=self.win.device_state.copy()
        for bad in (AUTO_STATE.replace('Auto:0','Auto:2'),AUTO_STATE.replace('FullMs:0','FullMs:3001'),AUTO_STATE.replace('Rate:0.0','Rate:nan')):
            self.win.worker._process_received_line(bad)
            self.assertEqual(self.win.device_state,before)
    def test_emergency_stop_cancels_auto_arm(self):
        self.win.worker._process_received_line(AUTO_STATE)
        self.win.btn_auto_edge.click()
        self.win.action_emergency_stop()
        self.assertEqual(self.queued(),['STOP'])
    def test_startup_auto_load_then_arm_after_confirmed_profile(self):
        self.win.startup_auto_edge=True
        self.win.worker._process_received_line(AUTO_STATE)
        app.processEvents()
        self.assertTrue(self.win.edge_profile_pending)
        self.assertTrue(self.win.startup_auto_edge)
        self.win.worker._process_received_line('OK BREF')
        self.win.worker._process_received_line(AUTO_STATE)
        app.processEvents()
        self.assertEqual(self.queued(),['BAUTO 1'])
    def test_startup_auto_cannot_override_emergency_stop(self):
        self.win.startup_auto_edge=True
        self.win.worker._process_received_line(AUTO_STATE)
        app.processEvents()
        self.win.worker._process_received_line('OK BREF')
        self.win.worker._process_received_line(AUTO_STATE)
        self.win.action_emergency_stop()
        app.processEvents()
        self.assertEqual(self.queued(),['STOP'])
        self.assertFalse(self.win.startup_auto_edge)
    def test_startup_auto_requires_alignment_and_calibration(self):
        self.win.startup_auto_edge=True
        self.win.worker._process_received_line(AUTO_STATE.replace('Cal:1','Cal:0'))
        app.processEvents()
        self.assertFalse(self.win.startup_auto_edge)
        self.assertEqual(self.queued(),[])
    def test_body_axes_need_confirmation(self):
        state=STATE.replace('App:0','App:1').replace('#', ',Ref:1,AxesOK:0,DirOK:0,Axis1:1,Axis2:2,Axis3:3#')
        self.win.worker._process_received_line(state)
        self.assertFalse(self.win.btn_balance_start.isEnabled())
        self.win.axis_combos[0].setCurrentIndex(1)
        self.win.axis_combos[1].setCurrentIndex(2)
        self.win.axis_combos[2].setCurrentIndex(0)
        self.win.action_send_axes()
        self.assertEqual(self.queued(), ['BAXIS 2 3 1'])
        self.win.worker._process_received_line(state.replace('AxesOK:0','AxesOK:1').replace('DirOK:0','DirOK:1'))
        self.assertTrue(self.win.btn_balance_start.isEnabled())
    def test_edge_countdown_reference_and_active_lock(self):
        state = STATE.replace('App:0','App:1').replace('EN:010','EN:000').replace('#', ',Ref:0,AxesOK:1,DirOK:1,AxisMask:1,DirMask:1,CountMs:0#')
        self.win.worker._process_received_line(state)
        self.assertFalse(self.win.btn_balance_countdown.isEnabled())
        self.assertFalse(self.win.btn_balance_start.isEnabled())
        self.win.worker._process_received_line(state.replace('Ref:0','Ref:1'))
        self.assertTrue(self.win.btn_balance_countdown.isEnabled())
        self.win.btn_balance_countdown.click()
        self.assertIn('BSTART 8', self.queued())
        self.win.worker._process_received_line(state.replace('Bal:0','Bal:5').replace('CountMs:0','CountMs:8000'))
        self.assertEqual(self.win.device_state['Bal'],5)
        self.assertIn('8.0',self.win.lbl_balance.text())
        self.assertFalse(self.win.btn_balance_countdown.isEnabled())
        self.win.worker._process_received_line(state.replace('Bal:0','Bal:6'))
        self.assertEqual(self.win.device_state['Bal'],6)
        self.assertIn('等待扶到',self.win.lbl_balance.text())
    def test_mode_change_cancels_pending_speed(self):
        self.win.on_motor_speed_change(2, 10)
        self.win._request_state_change('APP 1', {'App': 1})
        self.win._flush_speed_command()
        self.assertEqual(self.queued(), ['APP 1'])
        self.assertFalse(self.win.debounce_timer.isActive())
        self.win.worker._process_received_line(STATE.replace('App:0','App:1').replace('EN:010','EN:000'))
        self.assertFalse(self.win.card_m2.spin_speed.isEnabled())
        self.assertTrue(self.win.btn_balance_start.isEnabled())
    def test_configurable_trip_angle(self):
        state=STATE.replace('App:0','App:1').replace('#',',TripDeg:20.0#')
        self.win.worker._process_received_line(state)
        self.assertEqual(self.win.spin_trip_angle.value(),20)
        self.win.btn_trip_angle.click()
        self.assertEqual(self.queued(),['BAL 0','BTRIP 20.0'])
        self.win.worker.clear_pending()
        self.win.worker._process_received_line(state.replace('TripDeg:20.0','TripDeg:60.0'))
        self.assertEqual(self.win.spin_trip_angle.value(),60)
        self.win.btn_trip_angle.click()
        self.assertEqual(self.queued(),['BAL 0','BTRIP 60.0'])
        self.win.worker._process_received_line(state.replace('Bal:0','Bal:5'))
        self.assertFalse(self.win.btn_trip_angle.isEnabled())
    def test_stop_discards_queued_speed(self):
        self.win.worker.send_cmd('M2 20')
        self.win.on_motor_speed_change(2, 12)
        self.win.action_emergency_stop()
        self.win._flush_speed_command()
        self.assertEqual(self.queued(), ['STOP'])
        self.assertFalse(self.win.pending_flag)
    def test_balance_wheel_limit_readback_above_debug_speed_range(self):
        self.win.device_state=None
        state=STATE.replace('App:0','App:1').replace('EN:010','EN:000').replace('#',',VLimit:6.0,SLimit:330.0#')
        self.win.worker._process_received_line(state)
        self.assertEqual(self.win.spin_vlim.value(),330)
        self.assertEqual(self.win.card_m1.spin_speed.maximum(),180)
    def test_speed_only_selected_axis(self):
        self.win.on_motor_speed_change(2, -3)
        self.win._flush_speed_command()
        self.assertEqual(self.queued(), ['M2 -3.00'])
    def test_speed_mode_button_uses_ack_state(self):
        self.win.action_toggle_control_mode()
        self.assertEqual(self.queued(), ['MODE 1'])
        self.assertEqual(self.win.speed_mode, 0)
        self.win.worker._process_received_line('OK MODE:1')
        self.win.worker.clear_pending()
        self.win.action_toggle_control_mode()
        self.assertEqual(self.queued(), ['MODE 0'])
    def test_corner_requires_all_axes(self):
        state = STATE.replace('App:0','App:1').replace('BMode:1','BMode:4').replace('Aligned:111','Aligned:110')
        self.win.worker._process_received_line(state)
        self.assertFalse(self.win.btn_balance_start.isEnabled())
        self.win.worker._process_received_line(state.replace('Aligned:110','Aligned:111'))
        self.assertTrue(self.win.btn_balance_start.isEnabled())
    def test_balance_gain_readback(self):
        self.win.combo_gain_axis.setCurrentIndex(2)
        self.win.worker._process_received_line('OK LQR3: P=29.000, D=2.300, W=-0.0200, I=0.0010')
        self.assertAlmostEqual(self.win.balance_spins[2].value(), -.02)
    def test_old_and_extended_telemetry(self):
        received=[]
        self.win.worker.sig_telemetry.connect(received.append)
        base='$TELE,M1:0.00,M2:1.00,M3:0.00,Vq:5.00,EN:010,A1:0.00,A2:0.12,A3:0.00'
        self.win.worker._process_received_line(base+'#')
        full=base+',Gx:0.0,Gy:0.0,Gz:0.0,Ax:0.00,Ay:0.00,Az:1.00,H1:0.0,H2:1.2,H3:0.0,S1:0.00,S2:1.00,S3:0.00,Pit:0.00,Rol:0.00,Bal:0#'
        self.win.worker._process_received_line(full)
        self.assertEqual(len(received),2)
        self.assertTrue(received[1]['has_speed'])
        self.assertEqual(received[1]['bal'],0)
        self.assertEqual(len(self.win.history_s1),len(self.win.history_time))
    def test_quad_mode_state_and_count_labels(self):
        state = STATE.replace('#', ',EncType:1,C1:4096,C2:4000,C3:2048,Enc:111#')
        self.win.worker._process_received_line(state)
        self.assertEqual(self.win.device_state['EncType'], 1)
        self.assertEqual(self.win.device_state['C2'], 4000)
        self.assertIn('4096/4000/2048', self.win.lbl_balance.text())
        tele='$TELE,M1:0,M2:0,M3:0,Vq:5,EN:000,A1:0,A2:0,A3:0,H1:1,H2:2,H3:3#'
        self.win.worker._process_received_line(tele)
        self.assertEqual(self.win.card_m1.lbl_hall_status.text(), '[计数]')
    def test_invalid_state_is_ignored(self):
        before = self.win.device_state.copy()
        for bad in (STATE.replace('Dbg:2', 'Dbg:9'), STATE.replace('Tilt:0.12', 'Tilt:nan'), STATE.replace('Cal:1', 'Cal:3')):
            self.win.worker._process_received_line(bad)
        self.assertEqual(self.win.device_state, before)
    def test_installation_direction_does_not_override_user_edit(self):
        self.win.direction_combos[0].setCurrentIndex(1)
        self.win.worker._process_received_line(STATE.replace('#', ',D1:1,D2:1,D3:1#'))
        self.assertEqual(self.win.direction_combos[0].currentIndex(), 1)
        self.win.on_log_rx('OK BDIR')
        self.win.worker._process_received_line(STATE.replace('#', ',D1:-1,D2:1,D3:1#'))
        self.assertEqual(self.win.direction_combos[0].currentIndex(), 1)
    def test_tcp_eof_is_error(self):
        server=socket.socket();server.bind(('127.0.0.1',0));server.listen()
        link=ui.TcpLink('127.0.0.1',server.getsockname()[1])
        peer,_=server.accept();peer.close()
        deadline=time.monotonic()+1
        with self.assertRaises(ConnectionError):
            while time.monotonic()<deadline:
                link.in_waiting
                time.sleep(.01)
        link.close();server.close()
    def test_worker_heartbeat_and_disconnect(self):
        server=socket.socket();server.bind(('127.0.0.1',0));server.listen()
        received=[]
        def device():
            peer,_=server.accept();peer.settimeout(2);peer.sendall((STATE+'\r\n').encode())
            data=b''
            try:
                while b'STOP' not in data:
                    chunk=peer.recv(4096)
                    if not chunk: break
                    data+=chunk
            finally:
                received.append(data);peer.close()
        thread=threading.Thread(target=device);thread.start()
        worker=ui.SerialWorker();worker.connect_wifi('127.0.0.1',server.getsockname()[1])
        time.sleep(.55);worker.disconnect_port();thread.join(2);server.close()
        self.assertFalse(worker.isRunning())
        self.assertGreaterEqual(received[0].count(b'PING'),2)
        self.assertIn(b'STOP',received[0])
        self.assertFalse(worker.tx_queue)
if __name__=='__main__': unittest.main()

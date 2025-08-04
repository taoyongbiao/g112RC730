            if RC:
                roll, pitch, speed= read_vehicle_status_from_rc()
            else:
                roll, pitch, speed, wheel_slip, acc_g, suspension_travel, local_angular_vel = read_vehicle_status(ac_api) 
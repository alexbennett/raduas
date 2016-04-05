function [ ] = send_pid_update(throttle, set_point, Kp, Ki, Kd )
    global ser
    output_str = sprintf('%d %d %f %f %f', throttle, set_point, Kp, Ki, Kd);
    fprintf(ser, '%s', output_str);
end


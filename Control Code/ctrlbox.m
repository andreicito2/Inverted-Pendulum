% ctrlbox.m
%
%    Functions for communication with the inverted pendulum
%    ctrlbox interface via ethernet.
1;
%
% ctrlbox_init - initialize connection to ctrlbox
%
function rval = ctrlbox_init()
    global ctrlbox_con;
    global ctrlbox_err;

    ctrlbox_err = 0;

    ctrlbox_con = socket();
    sinfo = struct("addr","169.254.0.100", "port", 47820);
    rval = connect(ctrlbox_con,sinfo);

    return;
endfunction
%
% ctrlbox_send(cmdval,enable,period)
%  - send command value, enable, and sample period to ctrlbox
%    - cmdval = -32768 to +32767, where 32767=100% of DC bus voltage
%    - enable = 0 or 1
%    - period in usec
%
%  - future: measure time avg of pwm value, shutoff motor
%    if excessive.
function rval = ctrlbox_send(cmdval,enable,period)
    global ctrlbox_con;
    global ctrlbox_err;

    pwm = min(max(cmdval,-32000),32000);
    data = [pwm, 0, enable, period];
    try
        send(ctrlbox_con, typecast(int32(data(1:4)),'uint8'));
    catch
        ctrlbox_err = 1;
    end_try_catch

    rval = 0;
    return;
endfunction
%
% ctrlbox_recv - receive an array of four values from ctrlbox
%
function data = ctrlbox_recv()
    global ctrlbox_con;
    global ctrlbox_err;

    [rdata,len] = recv(ctrlbox_con,16);

    if (len ~= 16)
        fprintf('short data: %d\n', len);
        data = [0,0,0,0];
        ctrlbox_err = 2;
        return;
    end

    data = double(typecast(rdata,'int32'));
    return;

endfunction

%
% ctrlbox_error - check for a comm error
%
function val = ctrlbox_error()
    global ctrlbox_err;

    val = ctrlbox_err;

    return;
endfunction

%
% ctrlbox_shutdown - shutdown connection to ctrlbox
%
function ctrlbox_shutdown()
    global ctrlbox_con;
    global ctrlbox_err;

    ctrlbox_err = 0;

    % turn off motor
    send(ctrlbox_con,typecast(int32([0,0,0,0]),'uint8'));

    disconnect(ctrlbox_con);
    close(ctrlbox_con);
endfunction

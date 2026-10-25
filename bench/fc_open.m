function sp = fc_open(port)
%FC_OPEN  Open serial link to the flight controller (USB-Serial-JTAG).
%   sp = fc_open("COM5")
%   Baud rate is ignored by USB-JTAG but serialport requires one.
    arguments
        port (1,1) string
    end
    % close any stale handle on the same port
    old = serialportfind(Port=port);
    if ~isempty(old), delete(old); end
    sp = serialport(port, 115200);
    configureTerminator(sp, "LF");
    sp.Timeout = 2;
    flush(sp);
end

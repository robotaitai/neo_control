#include "CommProtocol.h"

HardwareSerial *CommProtocol::_up = nullptr;

void CommProtocol::begin(HardwareSerial &port, uint32_t baud)
{
    _up = &port;
    _up->begin(baud);
}

void CommProtocol::sendState(const State &s)
{
    // Format: <S,t,fl_pos,rl_pos,fl_vel,rl_vel>
    _up->printf("<S,%lu,%ld,%ld,%.2f,%.2f>\n",
                s.timestamp, s.fl_pos, s.rl_pos, s.fl_vel, s.rl_vel);
}

bool CommProtocol::receiveCmd(Command &cmd)
{
    // Expect: <C,fl_set,rl_set>\n
    static String buf;
    while (_up->available())
    {
        char c = _up->read();
        buf += c;
        if (c == '\n')
        {
            if (buf.startsWith("<C") && buf.endsWith(">\n"))
            {
                float a, b;
                if (sscanf(buf.c_str(), "<C,%f,%f>", &a, &b) == 2)
                {
                    cmd.fl_set = a;
                    cmd.rl_set = b;
                    buf = "";
                    return true;
                }
            }
            buf = ""; // discard garbage
        }
    }
    return false;
}

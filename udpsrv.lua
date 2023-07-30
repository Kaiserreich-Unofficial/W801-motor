local sys = require "sys"

local udpsrv = {}

function udpsrv.create(port, topic, adapter)
    local srv = {}
    srv.rxbuff = zbuff.create(1500)
    local sc = socket.create(adapter, function(sc, event)
        if event == socket.EVENT then
            local rxbuff = srv.rxbuff
            while 1 do
                local succ, data_len = socket.rx(sc, rxbuff)
                if succ and data_len and data_len > 0 then
                    local resp = rxbuff:toStr(0, rxbuff:used())
                    rxbuff:del()
                    sys.publish(topic, resp)
                else
                    break
                end
            end
        end
    end)
    if sc == nil then
        return
    end
    srv.sc = sc
    socket.config(sc, port, true)
    if socket.connect(sc, "255.255.255.255", 0) then
        srv.send = function(self, data, ip, port)
            if self.sc and data then
                return socket.tx(self.sc, data, ip, port)
            end
        end
        srv.close = function(self)
            socket.close(self.sc)
            -- sys.wait(200)
            socket.release(self.sc)
            self.sc = nil
        end
        return srv
    end
    socket.close(sc)
    socket.release(sc)
end

return udpsrv

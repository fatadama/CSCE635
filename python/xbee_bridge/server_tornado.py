import tornado.httpserver
import tornado.websocket
import tornado.ioloop
import tornado.web
import socket
import ipcPacker
import sys
'''
This is a super hacky websocket server that receives from Skywriter and forwards to the xbee bridge over nanomsg, then
takes nanomsg input from the bridge and forwards the string to Skywriter.
'''
ipc = ipcPacker.nanomsgClient()
class WSHandler(tornado.websocket.WebSocketHandler):

    def open(self):
        print 'new connection'

    ## We received a message from Matt: forward it to ipc
    def on_message(self, message):
        print 'message received:  %s' % message
        ipc.writeSocks(message)
        # read from IPC
        ipc.readSocks()
        # send anything in the send buffer
        print 'sending back message: %s' % ipc.msgFromBridge
        self.write_message(ipc.msgFromBridge)

    def on_close(self):
        print 'connection closed'
        sys.exit()

    def check_origin(self, origin):
        return True

application = tornado.web.Application([
    (r'/echo', WSHandler),
])


if __name__ == "__main__":
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen(9998)
    myIP = socket.gethostbyname(socket.gethostname())
    print '*** Websocket Server Started at %s***' % myIP
    tornado.ioloop.IOLoop.instance().start()

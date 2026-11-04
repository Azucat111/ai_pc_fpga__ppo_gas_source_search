"""A busy second controller must not send cleanup STOP to the first controller."""
import unittest
from unittest.mock import patch
from car.host import Client,ControlBusy
from test_host_keepalive import Clock,PLNetwork
from car.config import load_config

class BusyNetwork(PLNetwork):
    def __init__(self,clock):
        super().__init__(clock);self.busy=False
    def sendto(self,p,address):
        if p!=b'FCAR?' and p[16] in (1,2,3):
            self.tx.append((self.clock.now,p[16],int.from_bytes(p[4:8],'little')))
            self.busy=True
        else:super().sendto(p,address)
    def recvfrom(self,n):
        if self.busy:
            self.busy=False
            return b'FCAR_BUSY',('car',4242)
        return super().recvfrom(n)

class MultiClientTests(unittest.TestCase):
    def test_busy_refusal_does_not_stop_other_controller(self):
        clock=Clock();network=BusyNetwork(clock)
        with patch('car.host.time.monotonic',clock.monotonic):
            client=Client('car',load_config('config/car.json'),sock=network)
            client.receive()
            with self.assertRaisesRegex(ControlBusy,'Another station'):
                client.hold(2,100,100,2)
        self.assertEqual([m for _,m,_ in network.tx],[2])

    def test_stop_is_still_sent_explicitly(self):
        clock=Clock();network=BusyNetwork(clock)
        with patch('car.host.time.monotonic',clock.monotonic):
            client=Client('car',load_config('config/car.json'),sock=network)
            client.receive();state=client.send(0)
        self.assertEqual(state['last_action'],'stop')
        self.assertEqual([m for _,m,_ in network.tx],[0])

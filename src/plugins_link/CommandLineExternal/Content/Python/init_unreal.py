import unreal
import socket
import traceback
from threading import Thread
from queue import Queue


syslib = unreal.SystemLibrary()


class MyTimer:
	_hndl = None
	def __init__(self, *w, **kw):
		if w:self.start(*w, **kw)

	def start(self, foo, delay=0.0, repeat=False):
		self.stop()
		assert callable(foo)
		self._foo = foo
		self._delay = max(delay, 0.0)
		self._repeat = repeat
		self._passed = 0.0
		self._hndl = unreal.register_slate_post_tick_callback(self._tick)
	
	def stop(self):
		self._foo = None
		if not self._hndl: return
		unreal.unregister_slate_post_tick_callback(self._hndl)
		self._hndl = None

	def _tick(self, delta_time):
		if not self._foo: return self.stop()
		self._passed += delta_time
		if self._passed < self._delay: return
		try: self._foo()
		except: traceback.print_exc()
		if self._repeat:
			self._passed = 0.0
		else:
			self.stop()
			

class UDP2CMD:
	def __init__(self):
		self.port = 1234
		self._live = False
		self.from_sublime_queue = Queue()
			
	def start(self):
		args = unreal.SystemLibrary.get_command_line()
		tokens, switches, params = unreal.SystemLibrary.parse_command_line(args)
		if "tellunreal_listen_port" in params:
			print("tellunreal_listen_port:%r"%params["tellunreal_listen_port"])
			self.port = int(params["tellunreal_listen_port"])

		print("External Command Line object is initialized")
		if self._live: raise RuntimeError
		self._live = True
		self.listen_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.listen_socket.bind(("localhost", self.port))
		self.listen_thread = Thread(target = self._thr_listen)
		self.listen_thread.start()
		self._main_timer = MyTimer(self.onTimer, 0.3, repeat=True)

	def stop(self):
		print("STOP UDP2CMD %r"%self)
		self._live = False
		self._main_timer.stop()
		self.listen_socket.close() 

	def _thr_listen(self):
		while self._live:
			try:
				data, address = self.listen_socket.recvfrom(10000)
			except OSError as err:
				print("UDP2CMD _thr_listen recvfrom ERR:%s"%err)
				break

			if data:
				self.from_sublime_queue.put(data)
		print("UDP2CMD STOP thread %r"%self)

	def onTimer(self):
		if not self._live: 
			return self._main_timer.stop()
		if not self.from_sublime_queue.qsize(): return
		data = self.from_sublime_queue.get_nowait()
		print("UDP2CMD data:%r"%data)
		cmd = data.decode()
		syslib.print_string(None, string=cmd, print_to_screen=True, print_to_log=True, 
			text_color=[255.0, 0.0, 0.0, 255.0], duration=2.0) 
		syslib.execute_console_command(None, cmd)


prev = getattr(unreal, "udp_2_cmd_listener", None )
if prev:
	prev.stop()

unreal.udp_2_cmd_listener = UDP2CMD()
unreal.udp_2_cmd_listener.start()
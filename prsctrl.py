#!/usr/bin/env python

# prscrtl.py
#	A user interface for the PressureController2
#
# M. Hopcroft
#   hopcroft@reddogresearch.com
# Copyright 2016
#
# This software is distributed under the terms of the MIT License
#
# 	The MIT License (MIT) Copyright (c) 2016, Matthew A. Hopcroft
# 
# 	Permission is hereby granted, free of charge, to any person
# obtaining a copy of this software and associated documentation files
# (the "Software"), to deal in the Software without restriction,
# including without limitation the rights to use, copy, modify, merge,
# publish, distribute, sublicense, and/or sell copies of the Software,
# and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
# 
# 	The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
# 
# 	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
# BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
# ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

from __future__ import division
from __future__ import print_function
# add path for other components
import sys, os, time
# serial port library (pip install pyserial)
try:
	import serial
	import serial.tools.list_ports
except:
	print("")
	print("ERROR: The pyserial module cannot be imported (requires version 3+).")
	print(' Install the module with "pip install pyserial".')
	print("")
	sys.exit(1)
import glob
import curses
import curses.textpad as textpad

versionstr = "prsctrl v1.22"

# conversion factors for pressure
CountsPerkPa = 3.9605	# conversion factor for the Honeywell ASDXRRX015 sensor


def main():
	appStartTime = time.localtime()
	print('\n')
	print(versionstr)
	print('----')
	print(' Local time is ' + time.strftime("%Y/%m/%d-%H:%M:%S",appStartTime))
	print('')

	verbose = 1

	bytesize=8
	parity='N'
	stopbits=1
	timeout=3
	target_baudrate=115200

	port_name = get_port_name()
	if port_name is None:
		print("prsctrl: No controllers connected! (exit)")
		return

	s = serial.Serial(port_name, baudrate=target_baudrate, bytesize=bytesize, parity=parity, stopbits=stopbits, timeout=timeout)
	if verbose >= 1: print("Establishing communication with Controller at: " + s.name)

	# test connection to controller
	#  ping to accomodate auto-reset devices
	# Note: the "progress printing" sometimes doesn't work due to output buffering. Use python -u
	#  http://stackoverflow.com/questions/8031410/why-is-time-sleep-pausing-early
	send(s,'vr',verbose)
	time.sleep(0.5)
	while s.in_waiting==0:
		print(s.in_waiting, end=' ')
		time.sleep(1)
		send(s,'vr',verbose)
		time.sleep(1)
		
	s.reset_input_buffer()
	print("")
	verstr = sendcmd(s,'vr',verbose)	
	
	if verstr is None:
		print("prsctrl: Unable to connect to controller! (exit)")
		return

	print(verstr)


	# start up
	r = sendcmd(s,'db0',verbose)	# debug mode off

	# start screen function
	curses.wrapper(prsScreen,s,verbose)


####
# this function is the terminal window with menu of options
def prsScreen(stdscr,s,verbose=1):

	curses.noecho() # do not display keys when pressed
	curses.cbreak() # react to keypress

	# use a small window, vt100 compatible
	screen1 = stdscr.subwin(23,79,0,0) # height, width, originY, originX
	screen1.clear()
	screen1.nodelay(1)	# set getch() to non-blocking

	# get controller settings
	SensorZero = getval(s,'sz',verbose)	# get sensor zero value
	cmode = getval(s,'ct',verbose)	# are we controlling pressure?
	rmode = getval(s,'rn',verbose)	# are we running a waveform?
	setp = (getval(s,'tg',verbose) - SensorZero)/CountsPerkPa	# get pressure control setpoint
	prs = (float(sendcmd(s,'gp',verbose).split(',')[1]) - SensorZero)/CountsPerkPa	# get current pressure
	va = getval(s,'va',verbose)
	ve = getval(s,'ve',verbose)

	# initialize the menu items
	menu={}
	menu['version']=versionstr
	# set ascii graphic display of valve status
	if va==0 and ve==0:
		menu['valvepos']="S - | -+- | - V"
	elif va==255 and ve==0:
		menu['valvepos']="S - = -+- | - V"
	elif va==0 and ve==255:
		menu['valvepos']="S - | -+- = - V"
	elif va==255 and ve==255:
		menu['valvepos']="S - = -+- = - V"
	else:
		menu['valvepos']="S - X -+- X - V"
			
	if cmode == 0:
		menu['control']="[G] Start Control"
	else:
		menu['control']="[G] Stop Control"
		menu['valvepos']="S - C -+- C - V" # ascii graphic display of valve status

	menu['pressure']="Pressure: {:.1f} kPa".format(prs)
	menu['setpoint']="[P] Pressure Setpoint: {:.1f} kPa".format(setp)
	menu['loadwave']="[L] Load Wave File (0)"
	#menu['uploadwave']="[U] Upload to Controller"
	if rmode == 0:
		menu['runwave']="[R] Run Waveforms"
	else:
		menu['runwave']="[R] Stop Waveforms"
		menu['valvepos']="S - R -+- R - V" # ascii graphic display of valve status

	menu['valveopen']="[O] Open Valves"
	menu['valveclose']="[C] Close Valves"
	menu['valvesupply']="[S] Supply Pressure"
	menu['valvevent']="[V] Vent Pressure"
	menu['settings']="[Z] Settings"

	menu['quit']="[Q] Quit the Program"

	while True:
		screen1.border(0)

		# create the menu
		#   NB: coords are (y,x) from top left
		screen1.addstr(2, 2, menu['version'], curses.A_UNDERLINE)
		screen1.addstr(4, 3, "Make a Selection:")

		screen1.addstr(6, 3, "Pressure Control", curses.A_BOLD)
		screen1.addstr(7, 4, menu['setpoint'])
		screen1.addstr(7, 40, menu['control'])

		screen1.addstr(9, 3, "Pressure Waveforms", curses.A_BOLD)
		screen1.addstr(10, 4, menu['loadwave'])
		#screen1.addstr(11, 4, menu['uploadwave'])
		screen1.addstr(11, 4, menu['runwave'])

		screen1.addstr(13, 3, "Valve Operation", curses.A_BOLD)
		screen1.addstr(14, 4, menu['valveclose'])
		screen1.addstr(14, 30, menu['valveopen'])
		screen1.addstr(15, 4, menu['valvesupply'])
		screen1.addstr(15, 30, menu['valvevent'])

		screen1.addstr(18, 3, menu['settings'])

		screen1.addstr(20, 3, menu['quit'])

		# status display
		screen1.addstr(10, 50, menu['pressure'])
		screen1.addstr(12, 52, menu['valvepos'])
		screen1.addstr(11, 59, "|")

		screen1.addstr(4, 20, "") # put the cursor

		screen1.refresh() # redraw screen

		## Loop to update pressure on screen while we wait for keypresses
		kp = -1
		while kp == -1:
			kp = screen1.getch() # capture keypress
			time.sleep(0.2)
			# get new pressure reading and update window
			prs = (float(sendcmd(s,'gp',0).split(',')[1]) - SensorZero)/CountsPerkPa
			menu['pressure']="Pressure: {:.1f} kPa    ".format(prs)
			screen1.addstr(10, 50, menu['pressure'])
			screen1.addstr(4, 20, "") # put the cursor at bottom of text

			screen1.refresh() # redraw screen


		## Handle keypresses
		if kp == ord('G') or kp == ord('g'):	# control pressure
			cmode = getval(s,'ct',0)	# are we controlling pressure?
			if cmode == 0:
				r = sendcmd(s,'ct1',0)
				menu['control']="[G] Stop Control "
				menu['valvepos']="S - C -+- C - V"
			else:
				r = sendcmd(s,'ct0',0)
				menu['control']="[G] Start Control"
				menu['valvepos']="S - X -+- X - V"


		if kp == ord('P') or kp == ord('p'):	# pressure setpoint
			try:
				userinp = float(scrUserInp(stdscr,"Enter the Pressure Setpoint (kPa): "))
				cmd = "tg{:.0f}".format( (userinp * CountsPerkPa)+SensorZero )
				r = sendcmd(s,cmd,0)
				if r == 'OK':
					menu['setpoint']="[P] Pressure Setpoint: {:.1f} kPa   ".format(userinp)

				else:
					print("ERROR: Unable to set pressure setpoint!")
			except:
				continue


		if kp == ord('C') or kp == ord('c'):	# close valves
			r = sendcmd(s,'cl',0)
			if r == 'OK':
				menu['valvepos']="S - | -+- | - V"

		if kp == ord('O') or kp == ord('o'):	# open valves
			r = sendcmd(s,'op',0)
			if r == 'OK':
				menu['valvepos']="S - = -+- = - V"

		if kp == ord('S') or kp == ord('s'):	# supply valves
			r = sendcmd(s,'va255',0)
			r = sendcmd(s,'ve0',0)
			if r == 'OK':
				menu['valvepos']="S - = -+- | - V"

		if kp == ord('V') or kp == ord('v'):	# vent valves
			r = sendcmd(s,'ve255',0)
			r = sendcmd(s,'va0',0)
			if r == 'OK':
				menu['valvepos']="S - | -+- = - V"

		if kp == ord('L') or kp == ord('l'):	# load waveforms from file
			fname = scrUserInp(stdscr,"Enter the waveform filename: ")
			if os.path.isfile(fname):
				with open(fname,'r') as f:
					wavedesc = [line.rstrip('\n') for line in f]
				if wavedesc[0] != 'Waveform List':
					curses.beep()
				else:
					del wavedesc[0]
					# create and send a command for each waveform
					for w in wavedesc:
						cmd = parseWave(w)
						r = sendcmd(s,cmd,0)

					menu['loadwave']="[L] Load Wave File ({0})".format(len(wavedesc))
			else:
				curses.beep()
				scrMessage(stdscr,"ERROR: File not found")


		if kp == ord('R') or kp == ord('r'):	# run waveforms
			cmode = getval(s,'rn',0)	# are we running?
			if cmode == 0:
				r = sendcmd(s,'rn1',0)
				menu['runwave']="[R] Stop Waveforms "
				menu['valvepos']="S - R -+- R - V"
			else:
				r = sendcmd(s,'rn0',0)
				menu['runwave']="[R] Run Waveforms"
				menu['valvepos']="S - C -+- C - V"


		if kp == ord('Z') or kp == ord('z'):	# controller settings
			try:
				SensorZero = settingsWindow(stdscr,s)
			except:
				continue

		if kp == ord('Q'):	# quit program
			break

	curses.endwin() # call at program exit




####
# this function creates an userinp window
def scrUserInp(stdscr,prompt):
	"""Gets userinp from the user"""
	screen2 = curses.newwin(4,len(prompt)+4,10,int((78-len(prompt)+4)/2) ) # height, width, originY, originX
	curses.echo()
	screen2.clear()
	screen2.border(0)
	screen2.addstr(1, 2, prompt)
	screen2.refresh()
	userinp = screen2.getstr(2, 2, 25)
	del screen2
	curses.noecho()
	return userinp

####
# create a feedback window ("message dialog")
def scrMessage(stdscr,prompt):
	"""Display a message"""
	screen2 = curses.newwin(5,len(prompt)+4,10,20) # height, width, originY, originX
	screen2.clear()
	screen2.border(0)
	screen2.addstr(1, 2, prompt)
	screen2.addstr(3, max(int((len(prompt)+4)/2)-4,0), "<enter>")
	screen2.refresh()
	kp = screen2.getch() # capture keypress
	del screen2
	return


####
# create a settings window
def settingsWindow(stdscr,s):
	"""A sub-screen for the settings menu"""
	screen2 = curses.newwin(16,62,4,9) # height, width, originY, originX
	screen2.clear()
	screen2.border(0)
	#screen2.curs_set(0)
	pt = sendcmd(s,'pt',0).split(':')
	ph = sendcmd(s,'ph',0).split(':')
	vp = sendcmd(s,'vp',0).split(':')
	mv = getval(s,'mv',0)
	lp = getval(s,'lp',0)
	SensorZero = getval(s,'sz',0)
	menu = {}
	menu['pidhold']="[H] PID (Set&Hold) {0}".format(ph[1])
	menu['pidtrack']="[T] PID (Tracking) {0}".format(pt[1])
	menu['sensorzero']="[Z] Sensor Zero ({:.0f})".format(SensorZero)
	menu['sensorfilter']="[F] Sensor Filters (MAV {0}, LP {1})".format(mv,lp)
	menu['valveparam']="[V] Valve Parameters ({0})".format(vp[1])
	menu['saveset']="[S] Save Settings to Controller"

	while True:
		screen2.addstr(2, 2, "Controller Settings", curses.A_UNDERLINE)

		screen2.addstr(4, 2, menu['sensorzero'])
		screen2.addstr(5, 2, menu['sensorfilter'])

		screen2.addstr(7, 2, menu['pidhold'])
		screen2.addstr(8, 2, menu['pidtrack'])

		screen2.addstr(10, 2, menu['valveparam'])

		screen2.addstr(12, 2, menu['saveset'])
		screen2.addstr(13, 2, "[X] Exit settings")

		screen2.addstr(2, 21, "") # put the cursor

		screen2.refresh()
		kp = screen2.getch() # capture keypress

		# Menu handlers
		if kp == ord('Z') or kp == ord('z'):	# sensor zero
			userinp = int(scrUserInp(stdscr,"Enter the Sensor Zero value (counts): "))
			screen2.refresh()
			cmd = "sz{:.0f}".format(userinp)
			r = sendcmd(s,cmd,0)
			if r == 'OK':
				SensorZero = getval(s,'sz',0)
				menu['sensorzero']="[Z] Sensor Zero ({:.0f})  ".format(SensorZero)
			else:
				scrMessage(stdscr,"ERROR: Unable to set sensor zero value!")

		if kp == ord('F') or kp == ord('f'):	# sensor filter
			userinp = int(scrUserInp(stdscr,"Enter the Moving Average bin size (N): "))
			cmd = "mv{:.0f}".format(userinp)
			r = sendcmd(s,cmd,0)
			userinp = float(scrUserInp(stdscr,"Enter the LP cutoff frequency (Hz): "))
			cmd = "lp{:.1f}".format(userinp)
			r = sendcmd(s,cmd,0)
			mv = getval(s,'mv',0)
			lp = getval(s,'lp',0)
			menu['sensorfilter']="[F] Sensor Filter (MAV {0}, LP {1})    ".format(mv,lp)


		if kp == ord('H') or kp == ord('h'):	# PID hold
			userinp = scrUserInp(stdscr,"Enter the PID parameters for Set&Hold (Kp,Ki,Kd): ")
			cmd = "ph{0}".format(userinp)
			r = sendcmd(s,cmd,0)
			if r == 'OK':
				ph = sendcmd(s,'ph',0).split(':')
				menu['pidhold']="[H] PID (Set&Hold) {0}".format(ph[1])
			else:
				scrMessage(stdscr,"ERROR: Unable to set PID parameters")

		if kp == ord('T') or kp == ord('t'):	# PID track
			userinp = scrUserInp(stdscr,"Enter the PID parameters for Tracking (Kp,Ki,Kd): ")
			cmd = "pt{0}".format(userinp)
			r = sendcmd(s,cmd,0)
			if r == 'OK':
				pt = sendcmd(s,'pt',0).split(':')
				menu['pidtrack']="[T] PID (Tracking) {0}".format(pt[1])
			else:
				scrMessage(stdscr,"ERROR: Unable to set PID parameters")

		if kp == ord('V') or kp == ord('v'):	# valve parameters
			userinp = scrUserInp(stdscr,"Enter the Valve parameters (Smin,Sspan,Vmin,Vspan)): ")
			cmd = "vp{0}".format(userinp)
			r = sendcmd(s,cmd,0)
			if r == 'OK':
				vp = sendcmd(s,'vp',0).split(':')
				menu['valveparam']="[V] Valve Parameters ({0})    ".format(vp[1])
			else:
				scrMessage(stdscr,"ERROR: Unable to set PID parameters")

		if kp == ord('S') or kp == ord('S'):
			r = sendcmd(s,'sv',0)
			if r == 'OK':
				scrMessage(stdscr,"Settings saved to Controller flash memory")
			else:
				scrMessage(stdscr,"ERROR: Unable to save settings")


		if kp == ord('X') or kp == ord('x'):	# exit
			break

	del screen2
	return SensorZero


####
# send/read in one command
def sendcmd(s,cmd,verbose=1):
	"""sendcmd(s,cmd,verbose) sends a command a returns the response"""
	cmd = cmd.rstrip('\r\n')
	send(s,cmd,verbose)
	response = read(s,verbose)
	if verbose >=2: print("  {0}... {1}".format(cmd, response))
	return response

####
# return a numeric value
def getval(s,cmd,verbose=1):
	"""getval(s,cmd,verbose) returns a numeric value response"""
	cmd = cmd.rstrip('\r\n')
	send(s,cmd,verbose-1)
	response = read(s,verbose-1)
	rv = response.split(':')
	if verbose >=2: print("  {0}... {1}".format(cmd, rv[1]))
	try:
		return float(rv[1])
	except:
		return None

####
# read response strings from the serial port
def read(s,verbose=1):
	"""read(s,verbose) reads a response string from the serial device"""
	data = ''
	k=0
	while 1:
		data += s.read(size=1)
		k += 1 # count the number of bytes read
		if len(data)==0:
			if verbose >= 1: print("  read: timeout (" + str(s.timeout) + " sec)")
			return None

		pos = data.find('\r\n')
		if pos >= 0:
			line = data[:pos]
			if verbose >= 3: print('  read: received ' + str(len(line)) + ' bytes')
			if line.find(";")>=0 or line.find("RECV")>=0 or line.find("CTRL")>=0:
				k = 0
				pos = 0
				data = ''
				if verbose >= 3: print('  read: received "' + line + '"')
			else:
				return line

		# timeout if no terminator is received
		if k > 256:
			print("  read: max bytes read with no CR\LF (is data streaming?)")
			return ''

####
# send command strings to the serial port
def send(s,cmd,verbose=1):
	"""send(s,cmd,verbose) sends a command string to the serial device"""
	if verbose >= 3: print("  send command " + cmd)
	try:
		s.write(cmd + '\r\n')
		s.flush()
	except Exception as ex:
		print("send: There was an error in writing ( send() )")
		raise ex


####
# Get a list of serial port names
def get_port_name(devdir="/dev/",verbose=1):
	"""get_port_name(devdir,verbose) returns the name of the serial port for the controller.
	Assumes an Arduino using a virtual serial port. Duemilanove only supported."""
	
	# Note that we could use the pyserial tools.list_ports for more robust operation
	
	# determine OS
	from sys import platform as _platform
	if _platform == "linux" or _platform == "linux2":
		tty_str = ["ttyUSB*","ttyACM*"]

	elif _platform == "darwin":	# Mac OS X
		#tty_str = "tty.usbserial-*" # for Duemilanove. Need usbmodem for Uno
		tty_str = "tty.usb*" # generic match, will pick up other devices

	elif _platform == "win32":
		printf("ERROR: Windows is not supported")
		printf(" (But it would be easy for someone [who has a Windows computer] to do...")
		ports = list(serial.tools.list_ports.comports())
		for p in ports:
			print(p)
		sys.exit(1)

    # look for candidate USB devices. Return first candidate found.
	usbdev=[]
	for tty in tty_str:
		if verbose >= 3: print("  " + tty + " (" + _platform + ")" )
		usbdev+=glob.glob(devdir + tty)
	print("USB devices: " + str(usbdev) + " (" + str(len(usbdev)) + ")")
	if len(usbdev) > 0:
		portName = str(usbdev[0])
	else:
		print('get_port_name: No USB devices found in ' + devdir)
		portName = None

	return portName


####
# create a command string from a wave description
def parseWave(wavedesc):
	wave = wavedesc.split(',')
	# wave type
	if wave[0] == 'Constant':
		cmd = 'wc'
	elif wave[0] == 'Sinusoid':
		cmd = 'ws'
	elif wave[0] == 'Square':
		cmd = 'wq'
	elif wave[0] == 'Ramp':
		cmd = 'wr'

	# frequency
	fp = wave[1].split()
	if fp[1]=='Hz' or fp[1]=='HZ':
		if fp[0] == '0':
			wPer = 0
		else:
			wPer = round(1/float(fp[0])*1000) # period in ms
	else:
		wPer = float(fp[1])*1000
	cmd += str(wPer)

	# amplitude
	ap = wave[2].split()
	wAmp = float(ap[0]) * CountsPerkPa
	cmd += ','+str(wAmp)

	# offset
	ofs = wave[3].split()
	wOff = float(ofs[0]) * CountsPerkPa + SensorZero
	cmd += ','+str(wOff)

	# duty cycle
	dty = wave[4].split()
	cmd += ','+dty[0]

	# time
	tme = wave[5].split()
	wTime = float(tme[0]) * 60 * 1000
	cmd += ','+str(wTime)

	#print(cmd)
	return cmd



if __name__ == "__main__":
    main()

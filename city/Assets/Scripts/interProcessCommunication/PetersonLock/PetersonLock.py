import os
import sys
import traceback
import array

# for memory mappped files
import mmap
# for locks
import win32event
import win32con


"""
Class attributes are same to its C# counterpart
Python version > 3.5 or remove the typing used in functions
"""
class PetersonLock:
	def __init__(self, name: str, num_processes: int, existing: bool):
		self.name = name
		self.num_processes = num_processes
		self.hasAcquired = False
		
		if (existing):
			# open all MMFs - flags, turns, 
			self.procIdToAssign = mmap.mmap(0, 4, "Local\\" + "procIdToAssign_" + name)
			self.flags = mmap.mmap(0, 4 * num_processes, "Local\\" + "flags_" + name)
			self.turns = mmap.mmap(0, 4 * (num_processes - 1), "Local\\" + "turns_" + name)

			self.procIdMMFLock = win32event.OpenMutex(win32con.SYNCHRONIZE, False, "procIdMMFLock_" + name)
			win32event.WaitForSingleObject(self.procIdMMFLock, win32event.INFINITE)

			bytesProcId = self.procIdToAssign.read(4)
			intProcId = PetersonLock.bytesToInt(bytesProcId)

			# max process limit protection
			self.maxProc = mmap.mmap(0, 4, "Local\\" + "maxProc_" + name)
			self.maxProc.seek(0)
			intMaxProc = PetersonLock.bytesToInt(self.maxProc.read(4))
			if (intMaxProc == intProcId):
				raise Exception("Max proccess limit to initialize lock has already been reached")

			if (intMaxProc != self.num_processes):
				raise Exception("This lock isn't meant for the number of process it was initialized with")

			self.procId = intProcId
			# increment procIdToAssign for next process to see
			self.procIdToAssign.seek(0)
			self.procIdToAssign.write(PetersonLock.intToBytes(intProcId+1))

			win32event.ReleaseMutex(self.procIdMMFLock)
		else:
			raise NotImplementedError("Yet to implement for existing false")

		# Optimizations
		self.procIdBytes = PetersonLock.intToBytes(self.procId)
		self.bytesMinusOne = PetersonLock.intToBytes(-1)

		# could be replaced with some efficient array structure
		self.countBytesArray = [PetersonLock.intToBytes(i) for i in range(num_processes)]

	def acquire(self):
		if self.hasAcquired:
			raise Exception("Tried acquiring a lock that was already acquired")

		for count in range(self.num_processes - 1):
			# I think I'm in position "count" in the queue
			self.flags.seek(4 * self.procId)
			self.flags.write(self.countBytesArray[count])

			# and I'm the most recent process to think I'm in position "count"
			self.turns.seek(4 * count)
			self.turns.write(self.procIdBytes)

			# wait until
			while True:
				everyoneBehind = True
				# everyone thinks they're behind me
				for k in range(self.num_processes):
					if k != self.procId:
						self.flags.seek(4 * k)
						flags_k = self.flags.read(4)
						int_flag_k = PetersonLock.bytesToInt(flags_k)
						if (int_flag_k > count):
							# someone is ahead set to false
							everyoneBehind = False
							break
				
				if (everyoneBehind):
					break
				
				# or someone later than me thinks they're in position "count"
				self.turns.seek(4 * count)
				turns_count = self.turns.read(4)
				if (PetersonLock.bytesToInt(turns_count) != self.procId):
					break
			# now I can update my estimated position to "count"+1
		
		# now I'm at the head of the queue so I can start my critical section
		self.hasAcquired = True
	
	def release(self):
		if not self.hasAcquired:
			raise Exception("Tried releasing a lock that was never acquired")

		self.hasAcquired = False
		# tell everyone we are finished
		# I'm not in the queue anymore
		self.flags.seek(4 * self.procId)
		self.flags.write(self.bytesMinusOne)

	# Conversion methods int <-> bytes using standard types from python
	@staticmethod
	def intToBytes(integer: int) -> bytes:
		return integer.to_bytes(4, byteorder=sys.byteorder, signed=True)

	@staticmethod
	def bytesToInt(bytesBuffer: bytes) -> int:
		return int.from_bytes(bytesBuffer, byteorder=sys.byteorder, signed=True)
		
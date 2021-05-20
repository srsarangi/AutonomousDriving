using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO.MemoryMappedFiles;
using System.Threading;

/*
Conditions ensured: 
1. Convention for int is little endian
2. Since unity needs to start python processes implemntation to initialize lock is in C#, python assume require MMFs exist

Assumption: 
1. Byte writes and reads are atomic in main memory
2. Name of locks need to be different for different locks and mmf made shouldn't conflict with other part of code
*/
public class PetersonLock
{
	public string name;
	public int num_processes;
	public int procId;
	public bool hasAcquired;

	// stored bytes for optimization
	private byte[] procIdBytes;
	private List<byte[]> countBytesArray;
	private byte[] bytesMinusOne;

	// shared memory related vars
	private MemoryMappedFile procIdToAssign;
	private Mutex procIdMMFLock;
	private MemoryMappedFile flags;
	private MemoryMappedFile turns;
	private MemoryMappedFile maxProc;

	// corresponding accessors
	private MemoryMappedViewAccessor procIdToAssignAccessor;
	private MemoryMappedViewAccessor flagsAccessor;
	private MemoryMappedViewAccessor turnsAccessor;
	private MemoryMappedViewAccessor maxProcAccessor;

	public PetersonLock(string name, int num_processes, bool existing) {
		this.name = name;
		this.num_processes = num_processes;
		this.hasAcquired = false;

		// create flag and turn array and assign proc Id;
		if (existing) {
			throw new NotImplementedException("Yet to implement for existing: true");
		} else {
			float num_proc = (float) num_processes;
			// 4 bytes per int
			this.flags = MemoryMappedFile.CreateNew("flags_" + name, 4 * num_processes);
			this.turns = MemoryMappedFile.CreateNew("turns_" + name, 4 * (num_processes - 1));

			// create space to keep Index for next procId, space enough for an int
			this.procIdToAssign = MemoryMappedFile.CreateNew("procIdToAssign_" + name, 4);
			// procIdMMF needs to be porotected by system locks
			procIdMMFLock = new Mutex(true, "procIdMMFLock_" + name);
			this.procId = 0;

			// create all accessors
			this.procIdToAssignAccessor = procIdToAssign.CreateViewAccessor();
			this.flagsAccessor = flags.CreateViewAccessor();
			this.turnsAccessor = turns.CreateViewAccessor();

			// write the next ProcId to be assigned into procIdMMF
			// bytes format for int big endian implemented below
			byte[] buffer = PetersonLock.intToBytes(1);

			procIdToAssignAccessor.WriteArray(0, buffer, 0, buffer.Length);

			procIdMMFLock.ReleaseMutex();

			// protection against more processes than limit trying to initiate same lock
			this.maxProc = MemoryMappedFile.CreateNew("maxProc_" + name, 4);
			this.maxProcAccessor = maxProc.CreateViewAccessor();
			maxProcAccessor.WriteArray(0, PetersonLock.intToBytes(this.num_processes), 0, 4);
		}

		// Some information maintained in lock to optimize acquiring lock
		// i.e. not wasting cycles in converting int to bytes again and again
		this.procIdBytes = PetersonLock.intToBytes(this.procId);
		this.bytesMinusOne = PetersonLock.intToBytes(-1);

		this.countBytesArray = new List<byte[]>();
		for (int i = 0; i < num_processes; i++) {
			this.countBytesArray.Add(PetersonLock.intToBytes(i));
		}
	}

	// reference - https://cs.stackexchange.com/questions/60857/understanding-n-process-petersons-algorithm
	public void acquire() {
		// Saftey check double acquire
		if (this.hasAcquired) {
			throw new Exception("Tried acquiring a lock that was already acquired");
		}

		byte[] turns_count = new byte[4];
		byte[] flags_k = new byte[4];

		for (int count = 0; count < this.num_processes - 1; count++) {
			// I think I'm in position "count" in the queue
			flagsAccessor.WriteArray(4 * this.procId, this.countBytesArray[count], 0, 4);
			// and I'm the most recent process to think I'm in position "count"
			turnsAccessor.WriteArray(4 * count, this.procIdBytes, 0, 4);

			// wait untill
			while (true) {
				// everyone thinks they're behind me 
				bool everyoneBehind = true;
				for (int k = 0; k < this.num_processes; k++) {
					if (k != this.procId) {
						flagsAccessor.ReadArray(4 * k, flags_k, 0, 4);
						if (PetersonLock.bytesToInt(flags_k) > count) {
							// someone is ahead set to false
							everyoneBehind = false;
							break;
						}
					}
				}
				if (everyoneBehind) {
					break;
				}
				
				// or someone later than me thinks they're in position "count"
				turnsAccessor.ReadArray(4 * count, turns_count, 0, 4);
				if (PetersonLock.bytesToInt(turns_count) != this.procId) {
					break;
				}
			}
			// now I can update my estimated position to "count"+1
		}
		// now I'm at the head of the queue so I can start my critical section 
		this.hasAcquired = true;
	}

	public void release() {
		// Safety check can't release
		if (!this.hasAcquired) {
			throw new Exception("Tried releasing a lock that was never acquired");
		}

		this.hasAcquired = false;
		// tell everyone we are finished
		// I'm not in the queue anymore
		flagsAccessor.WriteArray(4 * this.procId, this.bytesMinusOne, 0, 4);
		byte[] buffer = new byte[4];
	}

	public void dispose() {
		// Dispose all accessors before the MMFs
		procIdToAssignAccessor.Dispose();
		flagsAccessor.Dispose();
		turnsAccessor.Dispose();
		maxProcAccessor.Dispose();

		procIdMMFLock.Dispose();

		procIdToAssign.Dispose();
		flags.Dispose();
		turns.Dispose();
		maxProc.Dispose();
	}

	public static byte[] intToBytes(int integer) {
		return BitConverter.GetBytes(integer);
	}

	public static int bytesToInt(byte[] buffer) {
		return BitConverter.ToInt32(buffer, 0);
	}
}

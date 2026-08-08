#pragma once

#include "HostDeviceInterface.h"
#include "CudaVirtualMemory.h"



struct PointDataManager{

	shared_ptr<CudaVirtualMemory> vm_position = nullptr;
	shared_ptr<CudaVirtualMemory> vm_color = nullptr;
	shared_ptr<CudaVirtualMemory> vm_flags = nullptr;

	PointData data;

	PointDataManager(){
		vm_position   = CudaVirtualMemory::create();
		vm_color      = CudaVirtualMemory::create();
		vm_flags      = CudaVirtualMemory::create();

		data.position      = (vec3*)vm_position->cptr;
		data.color         = (uint32_t*)vm_color->cptr;
		data.flags         = (uint32_t*)vm_flags->cptr;
	}

	~PointDataManager(){
		// TODO: free memory
	}

	// commit sufficient physical memory for the given amount of splats
	void commit(uint64_t numSplats){
		vm_position  ->commit(sizeof(*data.position  ) * numSplats);
		vm_color     ->commit(sizeof(*data.color     ) * numSplats);
		vm_flags     ->commit(sizeof(*data.flags     ) * numSplats);

		// CudaVirtualMemory::commit() may re-reserve the virtual address range
		// when the requested size exceeds the current reservation, which
		// changes cptr. Re-sync the device pointers so they always point at
		// the live virtual address.
		data.position = (vec3*)vm_position->cptr;
		data.color    = (uint32_t*)vm_color->cptr;
		data.flags    = (uint32_t*)vm_flags->cptr;
	}

	uint64_t getGpuMemoryUsage(){
		uint64_t usage = 0;
		
		usage += vm_position->comitted;
		usage += vm_color->comitted;
		usage += vm_flags->comitted;
		
		return usage;
	}

};


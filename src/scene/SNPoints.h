#pragma once

#include <string>

#include "SceneNode.h"
#include "HostDeviceInterface.h"
#include "Points.h"
#include "PointsManagement.h"
#include "ProgressivePointData.h"

using std::string;

struct SNPoints : public SceneNode{

	shared_ptr<Points> points;
	PointDataManager manager;

	// Optional Skye-style progressive rendering data. Populated lazily after the
	// canonical PointData finishes uploading (see SplatEditor_update.h) when the
	// user has selected the progressive point renderer. Unused by the HQS path.
	ProgressivePointCloud progressive;
	bool progressiveInitialized = false;

	SNPoints(string name, shared_ptr<Points> points)
		: SceneNode(name)
	{
		this->points = points;
		this->manager.data.transform = points->world;
	}

	SNPoints(string name)
		: SceneNode(name)
	{
		this->points = make_shared<Points>();
		this->manager.data.transform = points->world;
	}

	uint64_t getGpuMemoryUsage(){
		return manager.getGpuMemoryUsage() + progressive.getGpuMemoryUsage();
	}

	Box3 getBoundingBox(){
		return {manager.data.min, manager.data.max};
	}

};
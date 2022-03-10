#pragma once
#include "RRTstar.h"
class InformedRRTstar : public RRTstar
{
public:
	void nextIter(std::list<Nodes> &nodes, std::list<obstacles*> obst);
	Nodes sample(float c_max);
	bool usingInformedRRTstar = false;
protected:
	std::list<Nodes*> sol_nodes;
};
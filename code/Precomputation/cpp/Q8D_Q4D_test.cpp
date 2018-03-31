#define _USE_MATH_DEFINES
#include <levelset/levelset.hpp>
#include <helperOC/helperOC.hpp>
#include <helperOC/DynSys/DynSys/DynSysSchemeData.hpp>
#include "Q8D_Q4D.hpp"
#include "Q8D_Q4D.cpp"
#include <cmath>
#include <numeric>
#include <functional>
#include <cfloat>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <cstring>

/**
	@brief Tests the Q8D_Q4D class by computing a reachable set and then computing the optimal trajectory from the reachable set.
	*/
int main(int argc, char *argv[])
{
	bool dump_file = false;
	if (argc >= 2) {
		dump_file = (atoi(argv[1]) == 0) ? false : true;
	}
	bool useTempFile = false;
	if (argc >= 3) {
		useTempFile = (atoi(argv[2]) == 0) ? false : true;
	}
	const bool keepLast = false;
	const bool calculateTTRduringSolving = false;
	levelset::DelayedDerivMinMax_Type delayedDerivMinMax = 
	  levelset::DelayedDerivMinMax_Disable;
	if (argc >= 4) {
		switch (atoi(argv[3])) {
			default:
			case 0:
			delayedDerivMinMax = levelset::DelayedDerivMinMax_Disable;
			break;
			case 1:
			delayedDerivMinMax = levelset::DelayedDerivMinMax_Always;
			break;
			case 2:
			delayedDerivMinMax = levelset::DelayedDerivMinMax_Adaptive;
			break;
		}
	}

	bool useCuda = false;
	if (argc >= 5) {
		useCuda = (atoi(argv[4]) == 0) ? false : true;
	}
	int num_of_threads = 0;
	if (argc >= 6) {
		num_of_threads = atoi(argv[5]);
	}
	int num_of_gpus = 0;
	if (argc >= 7) {
		num_of_gpus = atoi(argv[6]);
	}
	size_t line_length_of_chunk = 1;
	if (argc >= 8) {
		line_length_of_chunk = atoi(argv[7]);
	}

	bool enable_user_defined_dynamics_on_gpu = true;
	if (argc >= 9) {
		enable_user_defined_dynamics_on_gpu = (atoi(argv[8]) == 0) ? false : true;
	}
	//!< Q8D_Q4D parameters
	const beacls::FloatVec initState{
  	  (FLOAT_TYPE)0, (FLOAT_TYPE)0, (FLOAT_TYPE)0, (FLOAT_TYPE)0};
  const beacls::FloatVec uRange{
  	  (FLOAT_TYPE)(-20./180.*M_PI), (20./180.*M_PI)};
	const beacls::FloatVec aRange{ (FLOAT_TYPE)(-1.), (FLOAT_TYPE)1. };
	const beacls::FloatVec dRange{ (FLOAT_TYPE)(-0.2), (FLOAT_TYPE)0.2 };
	helperOC::Q8D_Q4D* q8d_q4d = 
	  new helperOC::Q8D_Q4D(initState, uRange, aRange, dRange);

	const FLOAT_TYPE inf = std::numeric_limits<FLOAT_TYPE>::infinity();
	//!< Target and obstacle
	const beacls::FloatVec gMin{(FLOAT_TYPE)-2, (FLOAT_TYPE)-2, 
      (FLOAT_TYPE)(-35.*M_PI/180.), (FLOAT_TYPE)(-2*M_PI)};
	const beacls::FloatVec gMax{(FLOAT_TYPE)2, (FLOAT_TYPE)2, 
      (FLOAT_TYPE)(35.*M_PI/180.), (FLOAT_TYPE)(2*M_PI)};      
	levelset::HJI_Grid* g = helperOC::createGrid(gMin, gMax, 
      beacls::IntegerVec{65,65,41,35});
    //beacls::IntegerVec{35,35,25,19});
		

	const size_t numel = g->get_numel();
	const size_t num_dim = g->get_num_of_dimensions();

   // Position
  beacls::FloatVec targetp;
  targetp.assign(numel, 0);

  const beacls::FloatVec &ps = g->get_xs(0);
	std::transform(ps.cbegin(), ps.cend(), targetp.begin(), 
		  [](const auto& a) { return std::abs(a); });
	std::transform(targetp.cbegin(), targetp.cend(), targetp.begin(), 
		  std::negate<FLOAT_TYPE>());
	
   // Velocity
  beacls::FloatVec targetv;
  targetv.assign(numel, 0);

  const beacls::FloatVec &vs = g->get_xs(1);
	std::transform(vs.cbegin(), vs.cend(), targetv.begin(), 
		  [](const auto& a) { return std::abs(a); });
	std::transform(targetv.cbegin(), targetv.cend(), targetv.begin(), 
		  std::negate<FLOAT_TYPE>());
  std::transform(targetv.cbegin(), targetv.cend(), targetv.begin(), 
		  [](const auto& tv) { return tv / (FLOAT_TYPE)2; });

  // Overall cost
	std::vector<beacls::FloatVec> targets(1);
  targets[0].assign(numel, 0);
  std::transform(targetp.cbegin(), targetp.cend(), targetv.cbegin(), 
  	  targets[0].begin(), [](const auto& a, const auto& b) { 
  	  return std::min(a,b); });

	//!< Compute reachable set
	const FLOAT_TYPE tMax = 15;
	const FLOAT_TYPE dt = 0.2;
	beacls::FloatVec tau = generateArithmeticSequence<FLOAT_TYPE>(0., dt, tMax);

	// Dynamical system parameters
	helperOC::DynSysSchemeData* schemeData = new helperOC::DynSysSchemeData;
	schemeData->set_grid(g);
	schemeData->dynSys = q8d_q4d;
	schemeData->uMode = helperOC::DynSys_UMode_Max;
	schemeData->dMode = helperOC::DynSys_DMode_Min;

	// Target set and visualization
	helperOC::HJIPDE_extraArgs extraArgs;
	helperOC::HJIPDE_extraOuts extraOuts;
	extraArgs.targets = targets;

	extraArgs.execParameters.line_length_of_chunk = line_length_of_chunk;
	extraArgs.execParameters.calcTTR = calculateTTRduringSolving;
	extraArgs.keepLast = keepLast;
	extraArgs.execParameters.useCuda = useCuda;
	extraArgs.execParameters.num_of_gpus = num_of_gpus;
	extraArgs.execParameters.num_of_threads = num_of_threads;
	extraArgs.execParameters.delayedDerivMinMax = delayedDerivMinMax;
	extraArgs.execParameters.enable_user_defined_dynamics_on_gpu = 
	    enable_user_defined_dynamics_on_gpu;

	helperOC::HJIPDE* hjipde;
	hjipde = new helperOC::HJIPDE();

	beacls::FloatVec tau2;
	std::vector<beacls::FloatVec > datas;
	hjipde->solve(datas, tau2, extraOuts, targets, tau, schemeData, 
		  helperOC::HJIPDE::MinWithType_None, extraArgs);

  // save mat file
	std::string Q8D_Q4D_test_filename("Q8D_Q4D_test.mat");
	beacls::MatFStream* fs = beacls::openMatFStream(Q8D_Q4D_test_filename, 
		beacls::MatOpenMode_Write);

	if (dump_file) {
		beacls::IntegerVec Ns = g->get_Ns();

		g->save_grid(std::string("g"), fs);
		if (!datas.empty()) save_vector_of_vectors(datas, std::string("data"), Ns, false, fs);
		if (!tau2.empty()) save_vector(tau2, std::string("tau2"), Ns, false, fs);
	}

	beacls::closeMatFStream(fs);

	if (hjipde) delete hjipde;
	if (schemeData) delete schemeData;
	if (q8d_q4d) delete q8d_q4d;
	if (g) delete g;
	return 0;
}


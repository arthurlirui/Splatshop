#pragma once

#include <string>
#include <unordered_map>
#include <print>
#include <source_location>
#include <stack>

#include "unsuck.hpp"

#include "nvrtc.h"
#include <nvJitLink.h>
#include <cmath>
#include "cuda.h"

using std::string;

using namespace std;

#define NVJITLINK_SAFE_CALL(h,x)                                  \
do {                                                              \
   nvJitLinkResult result = x;                                    \
   if (result != NVJITLINK_SUCCESS) {                             \
      std::cerr << "\nerror: " #x " failed with error "           \
                << result << '\n';                                \
      size_t lsize;                                               \
      result = nvJitLinkGetErrorLogSize(h, &lsize);               \
      if (result == NVJITLINK_SUCCESS && lsize > 0) {             \
         char *log = (char*)malloc(lsize);                        \
         result = nvJitLinkGetErrorLog(h, log);                   \
         if (result == NVJITLINK_SUCCESS) {                       \
            std::cerr << "error: " << log << '\n';                \
            free(log);                                            \
         }                                                        \
      }                                                           \
      exit(1);                                                    \
   } else {                                                       \
      size_t lsize;                                               \
      result = nvJitLinkGetInfoLogSize(h, &lsize);                \
      if (result == NVJITLINK_SUCCESS && lsize > 0) {             \
         char *log = (char*)malloc(lsize);                        \
         result = nvJitLinkGetInfoLog(h, log);                    \
         if (result == NVJITLINK_SUCCESS) {                       \
            std::cerr << "info: " << log << '\n';                 \
            free(log);                                            \
         }                                                        \
      }                                                           \
      break;                                                      \
   }                                                              \
} while(0)

inline static void check(CUresult result, source_location location = source_location::current()){
	if(result != CUDA_SUCCESS){
		//cout << "cuda error code: " << result << endl;

		uint32_t code = result;
		string filename = location.file_name();
		uint32_t line = location.line();
		string functionName = location.function_name();

		const char* errorName;
		cuGetErrorName(result, &errorName);

		const char* errorString;
		cuGetErrorString(result, &errorString);

		println("ERROR(CUDA): code: {}, name: '{}', string: '{}'", code, errorName, errorString);
		println("    at file: {}, line: {}, function: {}", filename, line, functionName);

		exit(62350876);
	}
};

static void getArch(int& major, int& minor){
	CUdevice device;
	cuCtxGetDevice(&device);

	cuDeviceGetAttribute(&major, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR, device);
	cuDeviceGetAttribute(&minor, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MINOR, device);

	// major = 8;
	// minor = 6;
}

struct OptionalLaunchSettings{
	uint32_t gridsize = 0;
	uint32_t blocksize = 0;
	vector<void*> args;
	bool measureDuration = false;
	CUstream stream;
};

struct CudaModule{

	// static void cu_checked(CUresult result){
	// 	if(result != CUDA_SUCCESS){
	// 		cout << "cuda error code: " << result << endl;
	// 	}
	// };

	string path = "";
	string name = "";
	bool compiled = false;
	bool success = false;
	
	size_t ptxSize = 0;
	char* ptx = nullptr;

	size_t ltoirSize = 0;
	char* ltoir = nullptr;

	CudaModule(string path, string name){
		this->path = path;
		this->name = name;
	}

	void compile(){
		auto tStart = now();

		cout << "================================================================================" << endl;
		cout << "=== COMPILING: " << fs::path(path).filename().string() << endl;
		cout << "================================================================================" << endl;

		success = false;

		auto parent = fs::path(path).parent_path();
		string dir = parent.string();

		char* c_cuda_path = std::getenv("CUDA_PATH");
		string cuda_path = "";
		if(c_cuda_path){
			cuda_path = c_cuda_path;
		}

		if(cuda_path.size() == 0){
			println("Could not find cuda directory via environment variable 'CUDA_PATH'.");
			println("Trying '/usr/local/cuda/' instead.");
			println("If this does not work and you're on linux, try setting the CUDA_PATH variable like this, before starting the program.");
			println("export CUDA_PATH=/usr/local/cuda-12.8/");

			cuda_path = "/usr/local/cuda";
		}

		string optInclude = std::format("-I {}", dir).c_str();
		string cuda_include = std::format("-I {}/include", cuda_path);
		// CUDA 13.x relocated the CCCL/cuda/std headers: the Windows installer moves
		// them under include/cccl/cuda/std, while standard Linux installs keep the
		// traditional include/cuda/std layout. Probe before adding each path so we
		// only emit -I flags for directories that actually exist on this install
		// (NVRTC tolerates non-existent -I paths but warns noisily).
		string cudastd_include = "";
		string cccl_include = "";
		string cccl_cudastd_include = "";
		if(fs::exists(fs::path(cuda_path) / "include" / "cuda" / "std")){
			cudastd_include = std::format("-I {}/include/cuda/std", cuda_path);
		}
		if(fs::exists(fs::path(cuda_path) / "include" / "cccl")){
			cccl_include = std::format("-I {}/include/cccl", cuda_path);
			// In CUDA 13.x NVRTC no longer finds <cmath> etc. automatically on the
			// cccl layout; point directly at the CCCL cuda/std directory so that
			// standard headers like <cmath>, <type_traits> resolve to the CUDA
			// provided versions.
			if(fs::exists(fs::path(cuda_path) / "include" / "cccl" / "cuda" / "std")){
				cccl_cudastd_include = std::format("-I {}/include/cccl/cuda/std", cuda_path);
			}
		}


		CUdevice device;
		cuCtxGetDevice(&device);

		int major = 0;
		int minor = 0;
		getArch(major, minor);
		// cuDeviceGetAttribute(&major, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR, device);
		// cuDeviceGetAttribute(&minor, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MINOR, device);


		string arch = format("--gpu-architecture=compute_{}{}", major, minor);
		// string arch = "--gpu-architecture=compute_75";


		nvrtcProgram prog;
		string source = readFile(path);
		nvrtcCreateProgram(&prog, source.c_str(), name.c_str(), 0, NULL, NULL);


		std::vector<const char*> opts = {
			// "--gpu-architecture=compute_75",
			// "--gpu-architecture=compute_86",
			arch.c_str(),
			"--use_fast_math",
			"--extra-device-vectorization",
			"-lineinfo",
			cuda_include.c_str(),
			optInclude.c_str(),
			"-I ./",
			"-I ./include",
			// GCC predefined macros not available in NVRTC's EDG frontend
			"-D__FLT_MAX__=3.402823466e+38f",
			"--relocatable-device-code=true",
			"-default-device",                   // assume __device__ if not specified
			"--dlink-time-opt",                  // link time optimization "-dlto",
			// "--dopt=on",
			"--std=c++20",
			"--disable-warnings",
			"--split-compile=0",                 // compiler optimizations in parallel. 0 -> max available threads
			"--time=cuda_compile_time.txt",      // show compiler timings
		};
		// cuda/std and CCCL include paths are layout-dependent; only add the ones
		// that exist on the current toolkit install (see probe above).
		if(!cudastd_include.empty()) opts.push_back(cudastd_include.c_str());
		if(!cccl_include.empty()) opts.push_back(cccl_include.c_str());
		if(!cccl_cudastd_include.empty()) opts.push_back(cccl_cudastd_include.c_str());

		println("Compile Options: ");
		for(auto opt : opts){
			println("    {}", opt);
		}
		// cout << "====" << endl;

		nvrtcResult res = nvrtcCompileProgram(prog, opts.size(), opts.data());
		
		if (res != NVRTC_SUCCESS)
		{
			size_t logSize;
			nvrtcGetProgramLogSize(prog, &logSize);
			char* log = new char[logSize];
			nvrtcGetProgramLog(prog, log);
			//std::cerr << "Program Log: " <<  log << std::endl;
			println("Program Log: {}", log);

			delete[] log;

			if(res != NVRTC_SUCCESS && ltoir != nullptr){
				return;
			}else if(res != NVRTC_SUCCESS && ltoir == nullptr){
				println("failed gto compile {}. {}:{}", path, __FILE__, __LINE__);
				exit(123);
			}
		}

		nvrtcGetLTOIRSize(prog, &ltoirSize);
		ltoir = new char[ltoirSize];
		nvrtcGetLTOIR(prog, ltoir);

		cout << format("compiled ltoir. size: {} byte \n", ltoirSize);

		nvrtcDestroyProgram(&prog);

		compiled = true;
		success = true;

		printElapsedTime("compile " + name, tStart);
	}

};


struct CudaModularProgram{

	struct Timing{
		CudaModularProgram* program;
		string kernelName;
		double host_start;
		double host_duration;
		CUevent cuStart = 0;
		CUevent cuEnd = 0;
		double device_duration;
		int64_t frame;
		int launchIndex;
	};

	inline static bool measureTimings;
	inline static vector<Timing> timings;
	inline static mutex mtx_timings;

	struct CudaModularProgramArgs{
		vector<string> modules;
		vector<string> kernels;
	};

	// static void cu_checked(CUresult result){
	// 	if(result != CUDA_SUCCESS){
	// 		cout << "cuda error code: " << result << endl;
	// 	}
	// };

	vector<CudaModule*> modules;

	CUmodule mod;
	// CUfunction kernel = nullptr;
	void* cubin = nullptr;
	size_t cubinSize;

	vector<std::function<void(void)>> compileCallbacks;

	vector<string> kernelNames;
	unordered_map<string, CUfunction> kernels;
	
	struct EventPool {

		stack<CUevent> events;
		mutex mtx;

		CUevent acquire(){

			lock_guard<mutex> lock(mtx);

			if(events.size() == 0){
				CUevent event;
				cuEventCreate(&event, CU_EVENT_DEFAULT);
				events.push(event);
			}

			CUevent event = events.top();
			events.pop();

			return event;
		}

		void release(CUevent event){

			lock_guard<mutex> lock(mtx);

			events.push(event);
		}

	};
	inline static EventPool eventPool;
	// unordered_map<string, float> last_launch_duration;

	// int MAX_LAUNCH_DURATIONS = 50;
	// unordered_map<string, vector<float>> last_launch_durations;
	// unordered_map<string, int> launches_per_frame;

	CudaModularProgram(){

	}

	CudaModularProgram(vector<string> modules){
		construct({.modules = modules,});
	}

	CudaModularProgram(CudaModularProgramArgs args){
		construct(args);
	}

	static CudaModularProgram* fromCubin(void* cubin, int64_t size){
		CudaModularProgram* program = new CudaModularProgram();

		program->cubin = cubin;
		program->cubinSize = size;

		check(cuModuleLoadData(&program->mod, cubin));

		{ // Retrieve Kernels
			uint32_t count = 0;
			cuModuleGetFunctionCount(&count, program->mod);

			vector<CUfunction> functions(count);
			cuModuleEnumerateFunctions(functions.data(), count, program->mod);

			program->kernelNames.clear();

			for(CUfunction function : functions){
				const char* name;

				cuFuncGetName(&name, function);

				string strName = name;

				// println("============================================");
				// println("KERNEL: \"{}\"", strName);
				// int value;

				// cuFuncGetAttribute(&value, CU_FUNC_ATTRIBUTE_NUM_REGS, function);
				// cuFuncGetAttribute(&value, CU_FUNC_ATTRIBUTE_SHARED_SIZE_BYTES, function);

				program->kernelNames.push_back(strName);
				program->kernels[strName] = function;
			}
		}

		return program;
	}

	void construct(CudaModularProgramArgs args){
		vector<string> modulePaths = args.modules;
		// vector<string> kernelNames = args.kernels;

		// this->kernelNames = kernelNames;

		for(auto modulePath : modulePaths){

			string moduleName = fs::path(modulePath).filename().string();
			auto module = new CudaModule(modulePath, moduleName);

			module->compile();

			monitorFile(modulePath, [&, module]() {
				module->compile();
				link();
			});

			modules.push_back(module);
		}

		link();
	}

	void link(){

		cout << "================================================================================" << endl;
		cout << "=== LINKING" << endl;
		cout << "================================================================================" << endl;
		
		auto tStart = now();

		for(auto module : modules){
			if(!module->success){
				return;
			}
		}

		float walltime;
		constexpr uint32_t v_optimization_level = 1;
		constexpr uint32_t logSize = 8192;
		char info_log[logSize];
		char error_log[logSize];

		CUlinkState linkState;

		CUdevice cuDevice;
		cuDeviceGet(&cuDevice, 0);

		int major = 0;
		int minor = 0;
		getArch(major, minor);
		// cuDeviceGetAttribute(&major, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR, cuDevice);
		// cuDeviceGetAttribute(&minor, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MINOR, cuDevice);

		int arch = major * 10 + minor;
		// int arch = 86;
		string strArch = std::format("-arch=sm_{}", arch);

		// const char *lopts[] = {
		std::vector<const char*> lopts = { 
			"-dlto",      // link time optimization
			strArch.c_str(),
			"-time",
			"-verbose",
			"-O3",           // optimization level
			"-optimize-unused-variables",
			"-split-compile=0",
		};

		println("Link Options: ");
		for(auto opt : lopts){
			println("    {}", opt);
		}

		nvJitLinkHandle handle;
		nvJitLinkCreate(&handle, 2, lopts.data());

		for(auto module : modules){
			NVJITLINK_SAFE_CALL(handle, nvJitLinkAddData(handle, NVJITLINK_INPUT_LTOIR, (void *)module->ltoir, module->ltoirSize, module->name.c_str()));
		}

		NVJITLINK_SAFE_CALL(handle, nvJitLinkComplete(handle));
		NVJITLINK_SAFE_CALL(handle, nvJitLinkGetLinkedCubinSize(handle, &cubinSize));

		if(cubin){
			free(cubin);
			cubin = nullptr;
		}
		cubin = malloc(cubinSize);
		NVJITLINK_SAFE_CALL(handle, nvJitLinkGetLinkedCubin(handle, cubin));
		NVJITLINK_SAFE_CALL(handle, nvJitLinkDestroy(&handle));


		// static int cubinID = 0;
		// writeBinaryFile(format("./program_{}.cubin", cubinID), (uint8_t*)cubin, cubinSize);
		// cubinID++;
		
		println("cubinSize: {}", int64_t(cubinSize));
		println("cubin: {}", int64_t(cubin));

		check(cuModuleLoadData(&mod, cubin));

		{ // Retrieve Kernels
			uint32_t count = 0;
			cuModuleGetFunctionCount(&count, mod);

			vector<CUfunction> functions(count);
			cuModuleEnumerateFunctions(functions.data(), count, mod);

			kernelNames.clear();

			for(CUfunction function : functions){
				const char* name;

				cuFuncGetName(&name, function);

				string strName = name;

				// println("============================================");
				println("KERNEL: \"{}\"", strName);
				int value;

				// cuFuncGetAttribute(&value, CU_FUNC_ATTRIBUTE_NUM_REGS, function);
				// cuFuncGetAttribute(&value, CU_FUNC_ATTRIBUTE_SHARED_SIZE_BYTES, function);

				kernelNames.push_back(strName);
				kernels[strName] = function;
			}
		}

		for(auto& callback : compileCallbacks){
			callback();
		}

		printElapsedTime("link duration: ", tStart);

	}

	void onCompile(std::function<void(void)> callback){
		compileCallbacks.push_back(callback);
	}

	inline static void addTiming(Timing timing){
		lock_guard<mutex> lock(mtx_timings);

		if(timing.cuStart == 0 || timing.cuEnd == 0){
			println("Encounted empty events during event/time resolve. ");
			exit(1523254);
		}

		int launchIndex = 0;
		for(auto timing : timings){
			if(timing.kernelName == timing.kernelName){
				launchIndex++;
			}
		}

		timing.launchIndex = launchIndex;

		timings.push_back(timing);
	}

	inline static void resolveTimings(){

		cuCtxSynchronize();

		for(Timing& timing : timings){

			if(timing.cuStart == 0 || timing.cuEnd == 0){
				println("Encounted empty events during event/time resolve. ");
				exit(1523254);
			}

			float duration = 0.0f;
			cuEventElapsedTime(&duration, timing.cuStart, timing.cuEnd);

			timing.device_duration = duration;

			eventPool.release(timing.cuStart);
			eventPool.release(timing.cuEnd);
		}

	}

	inline static void clearTimings(){
		lock_guard<mutex> lock(mtx_timings);

		timings.clear();
	}

	void launch(string kernelName, vector<void*> args, OptionalLaunchSettings launchArgs = {}, source_location location = source_location::current()){
		void** _args = &args[0];

		this->launch(kernelName, _args, launchArgs, location);
	}

	void launch(string kernelName, void** args, OptionalLaunchSettings launchArgs, source_location location = source_location::current()){

		Timing timing;

		if(measureTimings){
			timing.cuStart = eventPool.acquire();
			cuEventRecord(timing.cuStart, launchArgs.stream);
			timing.host_start = now();
		}

		if(kernels.find(kernelName) == kernels.end()){
			println("ERROR: kernel {} does not exist in program", kernelName);
			exit(23753498);
		}

		auto res_launch = cuLaunchKernel(kernels[kernelName],
			launchArgs.gridsize, 1, 1,
			launchArgs.blocksize, 1, 1,
			0, launchArgs.stream, args, nullptr);


		if (res_launch != CUDA_SUCCESS) {

			// const char* str;
			// cuGetErrorString(res_launch, &str);

			const char* errorName;
			cuGetErrorName(res_launch, &errorName);

			const char* errorString;
			cuGetErrorString(res_launch, &errorString);

			println("ERROR: failed to launch kernel \"{}\". gridSize: {}, blocksize: {}", kernelName, launchArgs.gridsize, launchArgs.blocksize);

			uint32_t code = res_launch;
			string filename = location.file_name();
			uint32_t line = location.line();
			string functionName = location.function_name();

			println("ERROR(CUDA): code: {}, name: '{}', string: '{}'", code, errorName, errorString);
			println("    at file: {}, line: {}, function: {}", filename, line, functionName);

			exit(74532);
			// const char* str;
			// cuGetErrorString(res_launch, &str);
			// printf("error: %s \n", str);
			// cout << __FILE__ << " - " << __LINE__ << endl;
			// println("kernel: {}", kernelName);
		}

		if(measureTimings){
			timing.cuEnd = eventPool.acquire();
			cuEventRecord(timing.cuEnd, launchArgs.stream);
			
			// cuCtxSynchronize();

			// float duration;
			// cuEventElapsedTime(&duration, event_start, event_end);

			timing.host_duration = now() - timing.host_start;
			timing.device_duration = 0.0;
			timing.kernelName = kernelName;

			CudaModularProgram::addTiming(timing);

			// addLaunchDuration(kernelName, duration);
		}
	}

	void launch(string kernelName, vector<void*> args, int count, CUstream stream = 0, source_location location = source_location::current()){
		if(count == 0) return;

		void** _args = &args[0];

		this->launch(kernelName, _args, count, stream, location);
	}

	void launch(string kernelName, void** args, int count, CUstream stream = 0, source_location location = source_location::current()){

		if (count == 0){
			return;
		}

		if(kernels.find(kernelName) == kernels.end()){
			println("ERROR: kernel {} does not exist in program", kernelName);
		}


		uint32_t blockSize = 256;
		uint32_t gridSize = (count + blockSize - 1) / blockSize;

		Timing timing;
		if(measureTimings){
			timing.host_start = now();
			timing.cuStart = eventPool.acquire();
			cuEventRecord(timing.cuStart, stream);
		}

		auto res_launch = cuLaunchKernel(kernels[kernelName],
			gridSize, 1, 1,
			blockSize, 1, 1,
			0, stream, args, nullptr);

		if (res_launch != CUDA_SUCCESS) {
			// const char* str;
			// cuGetErrorString(res_launch, &str);

			const char* errorName;
			cuGetErrorName(res_launch, &errorName);

			const char* errorString;
			cuGetErrorString(res_launch, &errorString);

			println("ERROR: failed to launch kernel \"{}\". Threadcount: {}, gridSize: {}", kernelName, count, gridSize);

			uint32_t code = res_launch;
			string filename = location.file_name();
			uint32_t line = location.line();
			string functionName = location.function_name();

			println("ERROR(CUDA): code: {}, name: '{}', string: '{}'", code, errorName, errorString);
			println("    at file: {}, line: {}, function: {}", filename, line, functionName);

			exit(42415);
		}

		if(measureTimings){
			timing.cuEnd = eventPool.acquire();
			cuEventRecord(timing.cuEnd, stream);
			// cuCtxSynchronize();

			// float duration;
			// cuEventElapsedTime(&duration, event_start, event_end);

			timing.host_duration = now() - timing.host_start;
			timing.device_duration = 0.0;
			timing.kernelName = kernelName;

			CudaModularProgram::addTiming(timing);
		}
	}

	void launchCooperative(string kernelName, vector<void*> args, OptionalLaunchSettings launchArgs = {}, source_location location = source_location::current()){
		void** _args = &args[0];

		this->launchCooperative(kernelName, _args, launchArgs, location);
	}

	void launchCooperative(string kernelName, void** args, OptionalLaunchSettings launchArgs = {}, source_location location = source_location::current()){

		Timing timing;

		if(measureTimings){
			timing.host_start = now();
			timing.cuStart = eventPool.acquire();
			cuEventRecord(timing.cuStart, launchArgs.stream);
		}

		CUdevice device;
		int numSMs;
		cuCtxGetDevice(&device);
		cuDeviceGetAttribute(&numSMs, CU_DEVICE_ATTRIBUTE_MULTIPROCESSOR_COUNT, device);

		
		int blockSize = launchArgs.blocksize > 0 ? launchArgs.blocksize : 128;

		int numBlocks;
		CUresult resultcode = cuOccupancyMaxActiveBlocksPerMultiprocessor(&numBlocks, kernels[kernelName], blockSize, 0);
		numBlocks *= numSMs;
		
		//numGroups = 100;
		// make sure at least 10 workgroups are spawned)
		numBlocks = std::clamp(numBlocks, 10, 100'000);

		// if(launchArgs.blocksize > 0){
		// 	numBlocks = launchArgs.blocksize;
		// }

		auto kernel = this->kernels[kernelName];
		auto res_launch = cuLaunchCooperativeKernel(kernel,
			numBlocks, 1, 1,
			blockSize, 1, 1,
			0, launchArgs.stream, args);

		if (res_launch != CUDA_SUCCESS) {
			// const char* str;
			// cuGetErrorString(res_launch, &str);

			const char* errorName;
			cuGetErrorName(res_launch, &errorName);

			const char* errorString;
			cuGetErrorString(res_launch, &errorString);

			println("ERROR: failed to launchCooperative kernel \"{}\". gridSize: {}, blockSize: {}", kernelName, numBlocks, blockSize);

			uint32_t code = res_launch;
			string filename = location.file_name();
			uint32_t line = location.line();
			string functionName = location.function_name();

			println("ERROR(CUDA): code: {}, name: '{}', string: '{}'", code, errorName, errorString);
			println("    at file: {}, line: {}, function: {}", filename, line, functionName);

			exit(42415);
		}

		if(measureTimings){
			timing.cuEnd = eventPool.acquire();
			cuEventRecord(timing.cuEnd, launchArgs.stream);

			// cuCtxSynchronize();

			// float duration;
			// cuEventElapsedTime(&duration, event_start, event_end);

			timing.host_duration = now() - timing.host_start;
			timing.device_duration = 0.0;
			timing.kernelName = kernelName;

			CudaModularProgram::addTiming(timing);
		}
	}

};
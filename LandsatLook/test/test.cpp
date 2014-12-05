#include <LandsatLook.h>

//#define ENABLE_DATASET_1
//#define ENABLE_DATASET_2
#define ENABLE_DATASET_3

int main(int argc, char** argv) {

#ifdef ENABLE_DATASET_1
	std::string landsatPath = "C:\\Data\\Dev\\AgSolver-AlphaPixel\\alphapixel\\LandsatLook\\Data\\";
	std::string landsatFileRoot = "LC80260312014079LGN00_";
	bool exportNDVI = true;
	bool exportNDTI = true;
	double ULX = 476012.685;
	double ULY = 4658427.925;
	double LRX = 477569.273;
	double LRY = 4656871.337;
#elif ENABLE_DATASET_2
	std::string landsatPath = "C:\\Data\\Dev\\AgSolver-AlphaPixel\\alphapixel\\LandsatLook\\Data\\";
	std::string landsatFileRoot = "LC80250302014232LGN00_";
	bool exportNDVI = true;
	bool exportNDTI = true;
	double ULX = 576540.804;
	double ULY = 4755660.615;
	double LRX = 578071.417;
	double LRY = 4754189.928;
#else ENABLE_DATASET_3
	std::string landsatPath = "C:\\Data\\Dev\\AgSolver-AlphaPixel\\alphapixel\\LandsatLook\\Data\\";
	std::string landsatFileRoot = "LC80260312014191LGN00_";
	bool exportNDVI = true;
	bool exportNDTI = true;
	double ULX = 476012.685;
	double ULY = 4658427.925;
	double LRX = 477569.273;
	double LRY = 4656871.337;
#endif

	landsatlook::LandsatLook sc( exportNDVI, exportNDTI, landsatPath, landsatFileRoot, ULX, ULY, LRX, LRY );

	bool success = sc.Operate();

	return landsatlook::function();
}


#include <LandsatLook.h>

int main(int argc, char** argv) {

	std::string landsatPath = "C:\\Data\\Dev\\AgSolver-AlphaPixel\\alphapixel\\LandsatLook\\Data\\";
	std::string landsatFileRoot = "LC80260312014079LGN00_";
	bool exportNDVI = true;
	bool exportNDTI = true;

	landsatlook::LandsatLook sc( exportNDVI, exportNDTI, landsatPath, landsatFileRoot );

	sc.Operate();

	return landsatlook::function();
}


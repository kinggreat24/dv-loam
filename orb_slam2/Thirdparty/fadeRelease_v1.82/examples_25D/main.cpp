
#include <stdio.h>
#include <iostream>
#include <Fade_2D.h>

using namespace std;


// Functions defined in other *.cpp files (declaration avoids additional header files)
int terrain_main();
int advancedMeshing_main();
void segmentCheckerSimple_main();
void segmentCheckerTerrain_main();
int cutAndFill_main();
int removeBorderTriangles_main();

// Help

void info()
{
	GEOM_FADE25D::Fade_2D dt;
	dt.printLicense();
	
	cout<<"\n\n\n\tWelcome to the Fade 25D examples"<<endl;
	cout<<"\t-------------------------------"<<endl<<endl;
	cout<<"\t0...Terrain triangulation"<<endl;
	cout<<"\t1...Advanced meshing"<<endl;
	cout<<"\t2...Segment Checker (Simple Example)"<<endl;
	cout<<"\t3...Segment Checker (Terrain Example)"<<endl;
	cout<<"\t4...Cut & Fill"<<endl;
	cout<<"\t5...Remove Border Triangles"<<endl;
}

// Choose an example
int main()
{
	const int NUM_EXAMPLES(6);
	info();
	while(true)
	{
		char choice(0);
		cout << "\n\n\tChoose an example [0-"<<NUM_EXAMPLES-1<<"], q...quit, i...info: ";
		cin>>choice;

		cout<<"\n\n\t-------------------------------------------------"<<endl<<endl;
		switch(choice)
		{
			case '0': terrain_main();break;
			case '1': advancedMeshing_main();break;
			case '2': segmentCheckerSimple_main();break;
			case '3': segmentCheckerTerrain_main();break;
			case '4': cutAndFill_main();break;
			case '5': removeBorderTriangles_main();break;
			case 'q': return 0;
			case 'i': info();
			default: break;
		}
		cout<<"\n\t-------------------------------------------------"<<endl<<endl;
	}
}




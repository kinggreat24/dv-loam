
#include <stdio.h>
#include <iostream>
#include <Fade_2D.h>

using namespace std;

// Defined in the exampleX files
int example0_main();
int example1_main();
int example2_main();
int example3_main();
int example4_main();
int example5_main();
int example6_main();
int progress_main();
int facelist_main();
int randomData_main();

void info()
{
	GEOM_FADE2D::Fade_2D dt;
	dt.printLicense();

	cout<<"\n\n\n\tWelcome to the Fade 2D examples"<<endl;
	cout<<"\t-------------------------------"<<endl<<endl;
	cout<<"\t0...HelloTriangulation - a very simple code with visualization"<<endl;
	cout<<"\t1...Benchmark - single- and multithreaded Delaunay triangulation times"<<endl;
	cout<<"\t2...Traversing - Retrieve geometric elements and draw them"<<endl;
	cout<<"\t3...Constrained Delaunay - Insert constraint edges"<<endl;
	cout<<"\t4...Zones - Defined areas in triangulations"<<endl;
	cout<<"\t5...Zone operations - Boolean operations with zones"<<endl;
	cout<<"\t6...Delaunay Meshing - The mesh generator"<<endl;
	cout<<"\t7...Best Practices - Progress bar"<<endl;
	cout<<"\t8...Best Practices - Face list"<<endl;
	cout<<"\t9...Best Practices - Random data"<<endl;

}

int main()
{
	info();

	while(true)
	{
		char choice(0);
		const int NUM_EXAMPLES(10);
		cout << "\n\n\tChoose an example [0-"<<NUM_EXAMPLES-1<<"], q...quit, i...info: ";
		cin>>choice;

		cout<<"\n\n\t-------------------------------------------------"<<endl<<endl;
		switch(choice)
		{
			case '0': example0_main();break;
			case '1': example1_main();break;
			case '2': example2_main();break;
			case '3': example3_main();break;
			case '4': example4_main();break;
			case '5': example5_main();break;
			case '6': example6_main();break;
			case '7': progress_main();break;
			case '8': facelist_main();break;
			case '9': randomData_main();break;
			case 'q': return 0;
			case 'i': info();
			default: break;
		}
		cout<<"\n\t-------------------------------------------------"<<endl<<endl;
	}
}


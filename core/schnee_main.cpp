#include "vector.h"

#include <iostream>
#include <string>

int main(int argc, const char * argv[])
{
	if(argc < 3)
	{
		std::cout << argv[0] << " off_input_file k" << std::endl;
		exit(2);
	}

	// In off file
	std::string pin = argv[1];
	int k = std::stoi(argv[2]);

	return 0;
}

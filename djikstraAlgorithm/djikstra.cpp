#include <iostream>

void init(int * array);

int main (void) {
	int width;
	int height;
	std::cin >> width;
	std::cin >> height;
	int array[width][height];
	int size[2];
	size[0] = *(&array + 1) - array;
	size[1] =  *(&(array[1]) + 1) - array[1];
	std::cout << "Number of rows> " << size[0] << std::endl;
	std::cout << "Number of columns> " << size[1] << std::endl; 	
	init(array);
	return 0;
}

void init(int array[]) {
	std::cout << "Called successfully!\n";
}

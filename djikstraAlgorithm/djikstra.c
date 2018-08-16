#include <stdio.h>

void init(void ** array);

int main (void) {
	int width;
	int height;
	scanf("%d %d", &width, &height);
	int array[width][height];
	int size[2];
	size[0] = *(&array + 1) - array;
	size[1] =  *(&(array[1]) + 1) - array[1];
	printf("Number of rows> %d\n", size[0]);
	printf("Number of columns> %d\n", size[1]); 
	init((void*)array);
	return 0;
}

void init(void ** array) {
	
	int * a = *(array);
	printf("%p\n", a); 
	printf("Called successfully!\n");
}

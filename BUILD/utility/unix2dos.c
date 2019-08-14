//unix2dos

//

#include <stdio.h>

#include <stdlib.h>



int main(int argc, char* argv[])

{

	int ch;

	FILE* fpinPtr, * fpoutPtr;



	if (argc != 3)

	{

		printf("UNIX2DOS program.\n\n");

		printf("Usage: command source_file target_file\n");

		printf("Usage example: \"unix2dos src.txt obj.txt\"\n");

		exit(EXIT_FAILURE);

	}



	if ((fpinPtr = fopen(argv[1], "rb")) == NULL)

	{

		printf("Input file \"%s\" could not be opened\n", argv[1]);

		exit(EXIT_FAILURE);

	}



	if ((fpoutPtr = fopen(argv[2], "wb")) == NULL)

	{

		printf("Outout file \"%s\" could not be opened\n", argv[2]);

		exit(EXIT_FAILURE);

	}



	while (!feof(fpinPtr))

	{

		ch = fgetc(fpinPtr);

		if (ch > -1 && ch != '\n')

		{

			fputc(ch, fpoutPtr);

		}

		else if (ch > -1)

		{

			fputc('\r', fpoutPtr);

			fputc(ch, fpoutPtr);

		}



	}



	fclose(fpinPtr);

	fclose(fpoutPtr);



	return 0;

}


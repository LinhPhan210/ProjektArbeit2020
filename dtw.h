#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define MIN(a,b)            ((a)<(b)?(a):(b))
#define SMALLEST(a,b,c)     (MIN(MIN(a,b),c))

int dtwOneDimSameSize(int length, int8_t array1[], int8_t array2[]){
	int matrix[length][length];
    //
    matrix[0][0] = fabs(array1[0] - array2[0]);
    for(int i = 1; i < length; i++){
        matrix[0][i] = fabs(array1[0] - array2[i]) + matrix[0][i-1];
    }
    for(int i = 1; i < length; i++){
        matrix[i][0] = fabs(array1[i] - array2[0]) + matrix[i-1][0];
    }
    //
    for(int unitDiagonal = 1; unitDiagonal <length; unitDiagonal++){
        for(int i = unitDiagonal; i < length; i++){
            matrix[unitDiagonal][i] = fabs(array1[unitDiagonal] - array2[i])
                                    + SMALLEST(matrix[unitDiagonal-1][i],
                                                matrix[unitDiagonal-1][i-1],
                                                matrix[unitDiagonal][i-1]);
        }
        for(int i = unitDiagonal + 1; i < length; i++ ){
            matrix[i][unitDiagonal] = fabs(array1[i] - array2[unitDiagonal])
                                    + SMALLEST(matrix[i][unitDiagonal-1],
                                                matrix[i-1][unitDiagonal-1],
                                                matrix[i-1][unitDiagonal]);
        }
    }

    int result = matrix[length-1][length-1];

    return result;
}




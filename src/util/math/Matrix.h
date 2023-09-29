#include <vector>

class Matrix {
private:
    std::vector<std::vector<float>> matrix;

public:
   Matrix(int rows, int columns, float value) {
    matrix.resize(rows);
    for(auto & i : matrix) {
        i.resize(columns);
    }
    for(int i = 0; i < rows; i++) {
        for(int j = 0; j < columns; j++) {
            matrix.at(i).at(j) = value;
        }
    }
}

    explicit Matrix(std::vector<std::vector<float>> matrix) {
     this->matrix.resize(matrix.size());
     for(auto & i : matrix) {
         i.resize(matrix[0].size());
     }
     for(int i = 0; i < matrix.size(); i++) {
         for(int j = 0; j < matrix[0].size(); j++) {
             matrix.at(i).at(j) = value;
         }
     }
 }



/**
 * Multiplies this matrix with another matrix.
 * @param other The matrix to multiply with.
 * @return The resulting matrix object.
 */

void set(int row, int col, float value);

float get(int row, int col) const;

int rows() const;

int columns() const;

Matrix divide(const Matrix& other) const;

Matrix multiply(const Matrix& other) const;

Matrix add(const Matrix& other) const;

Matrix getTranspose() const;

};
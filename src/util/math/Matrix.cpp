#include "Matrix.h"


Matrix Matrix::divide(const Matrix& other) const {
        if(other.columns() != columns() || other.rows() != rows()) {
        throw std::invalid_argument("Both matrices must be of the same dimensions");
    }
    Matrix result(rows(), columns(), 0.0f);
    for(int row = 0; row < rows(); row++) {
        for(int col = 0; col < columns(); col++) {
            result.set(row, col, get(row, col) / other.get(row, col));
        }
    }
    return result;
}

Matrix Matrix::multiply(const Matrix& other) const {
    if(other.matrix.size() != matrix.at(0).size()) {
        throw std::invalid_argument("This matrix columns must be equal to other matrix rows");
    }

    int rows = this->rows();
    int columns = other.columns();
    Matrix result(rows, columns, 0.0f);

    for(int i = 0; i < rows; i++) {
        for(int j = 0; j < columns; j++) {
            for(int k = 0; k < matrix[0].size(); k++) {
                result.matrix[i][j] += matrix[i][k] * other.matrix[k][j];
            }
        }
    }

    return result;
}

Matrix Matrix::add(const Matrix& other) const {
    if(rows() != other.rows() || columns() != other.columns()) {
        throw std::invalid_argument("Both arrays must be of the same dimensions for addition.");
    }

    Matrix result(rows(), columns(), 0.0f);
    for(int row = 0; row < rows(); row++) {
        for(int col = 0; col < columns(); col++) {
            result.set(row, col, get(row, col) + other.get(row, col));
        }
    }

    return result;
}

Matrix Matrix::getTranspose() const {
    Matrix result(rows(), columns(), 0.0f);
    for(int row = 0; row < rows(); row++) {
        for(int col = 0; col < columns(); col++) {
            result.set(col, row, get(row, col));
        }
    }
    
    return result;
}

int Matrix::columns() const {
    return (int) this->matrix[0].size();
}

int Matrix::rows() const {
    return (int) this->matrix.size();
}

void Matrix::set(int row, int col, float value)  {
    matrix[row][col] = value;
}

float Matrix::get(int row, int col) const {
    return matrix[row][col];
}
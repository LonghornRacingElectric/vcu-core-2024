#include "Matrix.h"

Matrix Matrix::getTranspose() const {
    Matrix result(columns(), rows(), 0.0f);
    for(int row = 0; row < rows(); row++) {
        for(int col = 0; col < columns(); col++) {
            result.set(col, row, get(row, col));
        }
    }
    
    return result;
}

int Matrix::columns() const {
    return (int) this->con[0].size();
}

int Matrix::rows() const {
    return (int) this->con.size();
}

void Matrix::set(int row, int col, float value)  {
    con.at(row).at(col) = value;
}

float Matrix::get(int row, int col) const {
    return con[row][col];
}

Matrix Matrix::divide(const Matrix& other) const {
    if(other.columns() != columns() || other.rows() != rows()) {
        throw std::invalid_argument("Both matrices must be of the same dimensions");
    }
    Matrix result(rows(), columns(), 0.0f);
    for(int row = 0; row < rows(); row++) {
        for(int col = 0; col < columns(); col++) {
            if(other.get(row, col) == 0 && get(row, col) == 0) {
                result.set(row, col, 0);
            } else {
            result.set(row, col, get(row, col) / other.get(row, col));
                }
        }
    }
    return result;
}

Matrix Matrix::multiply(const Matrix& other) const {
    if(columns() != other.rows()) {
        throw std::invalid_argument("This matrix columns must be equal to other matrix rows");
    }

    int rows = this->rows();
    int columns = other.columns();
    Matrix result(this->rows(), other.columns(), 0.0f);

    for(int i = 0; i < rows; i++) {
        for(int j = 0; j < columns; j++) {
            for(int k = 0; k < con[0].size(); k++) {
                result.con[i][j] += con[i][k] * other.con[k][j];
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

Matrix Matrix::subtract(const Matrix &other) const {
    if(rows() != other.rows() || columns() != other.columns()) {
        throw std::invalid_argument("Both arrays must be of the same dimensions for addition.");
    }

    Matrix result(rows(), columns(), 0.0f);
    for(int row = 0; row < rows(); row++) {
        for(int col = 0; col < columns(); col++) {
            result.set(row, col, get(row, col) - other.get(row, col));
        }
    }

    return result;
}

float Matrix::determinant() const {
 return 1.0f;
}

Matrix Matrix::operator*(const Matrix &other) const {
    return multiply(other);
}

Matrix Matrix::operator+(const Matrix &other) const {
    return add(other);
}

Matrix Matrix::operator/(const Matrix &other) const {
    return divide(other);
}

Matrix Matrix::operator-(const Matrix &other) const {
    return subtract(other);
}

Matrix Matrix::getIdentityMatrix(int n) {
    Matrix result(n,n);
    for(int row = 0; row < n; row++) {
        result.set(row, row, 1);
    }
    return result;
};

std::string Matrix::toString() {
    std::string base;
    for(int row = 0; row < rows(); row++) {
        base += "| ";
        for(int col = 0; col < columns(); col++) {
            base += std::to_string(get(row, col)) + " ";
        }
        base += "|\n";
    }
    return base;
}
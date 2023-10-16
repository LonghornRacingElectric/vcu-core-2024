#include <vector>
#include <string>

class Matrix {
private:
    typedef std::vector<std::vector<float>> matrix;
    std::vector<std::vector<float>> con;

    int getNumLength(int number){
        if(number / 10 == 0){
            if(number < 0){
                return 2;
            }
            return 1;
        }
        return 1 + getNumLength(number / 10);
    }

public:
   Matrix(int rows, int columns, float value) {
    con = matrix(rows, std::vector<float>(columns));
    for(int i = 0; i < rows; i++) {
        for(int j = 0; j < columns; j++) {
            con.at(i).at(j) = value;
        }
    }
}

    /**
     * Initializes a new Matrix of size rows x columns with default values of 0
     * @param rows
     * @param columns
     */
    Matrix(int rows, int columns) {
        con = matrix(rows, std::vector<float>(columns));
        for(int i = 0; i < rows; i++) {
            for(int j = 0; j < columns; j++) {
                con.at(i).at(j) = 0;
            }
        }
    }

    explicit Matrix(std::vector<std::vector<float>> matrix) {
     this->con.resize(matrix.size());
     for(auto & i : matrix) {
         i.resize(matrix[0].size());
     }
     for(int i = 0; i < matrix.size(); i++) {
         for(int j = 0; j < matrix[0].size(); j++) {
             matrix.at(i).at(j) = 0;
         }
     }
 }

 Matrix() {
       this->con.resize(0);
   }

static Matrix getIdentityMatrix(int n);



    /**
 * Multiplies this matrix with another matrix.
 * @param other The matrix to multiply with.
 * @return The resulting matrix object.
 */

void set(int row, int col, float value);

float get(int row, int col) const;

int rows() const;

int columns() const;

/**
 * Technically, Matrices cannot be divided -- in this context,
 * we're just dividing each value of the matrix with a corresponding
 * value in the other matrix (works as multiplying by the inverse of a matrix
 * for a determinant matrix)
 * @param other
 * @return a resulting matrix with applied divisions
 */
Matrix divide(const Matrix& other) const;

Matrix multiply(const Matrix& other) const;

Matrix add(const Matrix& other) const;

Matrix subtract(const Matrix& other) const;

Matrix inverse(const Matrix& other) const;

Matrix getTranspose() const;

float determinant() const;

Matrix operator*(const Matrix& other) const;

Matrix operator+(const Matrix& other) const;

Matrix operator-(const Matrix& other) const;

/**
 * Technically, Matrices cannot be divided -- in this context,
 * we're just dividing each value of the matrix with a corresponding
 * value in the other matrix (works as multiplying by the inverse of a matrix
 * for a determinant matrix)
 * @param other
 * @return a resulting matrix with applied divisions
 */
Matrix operator/(const Matrix& other) const;

std::string toString();
};
import java.util.Random;

import org.ejml.factory.SingularMatrixException;
import org.ejml.simple.SimpleMatrix;

/**
 * This class is to demonstrate the use of the linear algebra libraray EJML for solving a linear system
 * 
 * @author cxz
 */
public class LinearSolveTest {
    /**
     * A simple test of solving Ax = b.
     * 
     * Most of the code here follows:
     * https://code.google.com/p/efficient-java-matrix-library/wiki/SolvingLinearSystems
     */
    public void test1() {
        SimpleMatrix A = new SimpleMatrix(5,5);
        SimpleMatrix b = new SimpleMatrix(5,1);
        
        Random r = new Random();
        // set the element of A
        // for demonstration, we generate a random matrix
        for(int i = 0;i < 5;++ i) {  // row
            for(int j = 0;j < 5;++ j) { // column
                A.set(i,j,r.nextDouble());
            }
            b.set(i,0,1);
        }
        System.out.println("A = " + A);
        System.out.println("b = " + b);
        
        // do the linear solve Ax = b
        try {
            SimpleMatrix x = A.solve(b);
            System.out.println("solution x = " + x);
            
            // verify the solution: Ax - b = 0
            SimpleMatrix res = A.mult(x).minus(b);
            System.out.println("residual = " + res);
            
            // due the the numerical error norm(Ax - b) might not be exact zero, 
            // but it should be very small
            if ( res.normF() > 1E-8f ) {
                throw new RuntimeException("The linear solve has problems!");
            }
        } catch ( SingularMatrixException e ) {
            throw new IllegalArgumentException("Singular matrix");
        }
    }
    
    public static void main(String[] args) {
        LinearSolveTest tester = new LinearSolveTest();
        
        // repeat the tests 5 times
        for(int i = 0;i < 5;++ i) {
            System.out.println("================= test " + i + " ================");
            tester.test1();
        }
    }
}
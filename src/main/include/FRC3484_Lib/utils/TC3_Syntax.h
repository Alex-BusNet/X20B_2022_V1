#ifndef TC3_SYNTAX_H
#define TC3_SYNTAX_H

#include <cstdarg>
#include <cmath>

using namespace std;

#define ALLOW_TC_IF_SYNTAX  // TwinCAT IF Syntax (IF..THEN..ELSE/ELSIF THEN...END_IF)
#define ALLOW_TC_MATH       // Allow TwinCAT math operators (ADD, SUB, MULT, DIV, EXPT; EXP is part of cmath)
#define ALLOW_TC_COMPARE    // Allow TwinCAT comparison operators (LE, LT, EQ, NE, GE, GT)
#define ALLOW_TC_BITWISE    // Allow TwinCAT bitwise operators (SHL, SHR, XOR, COMP)

#if defined(ALLOW_TC_IF_SYNTAX)
    #define IF              if(
    #define THEN            ){
    #define ELSE            } else {
    #define ELSIF           } else if(
    #define END_IF          }

    #define SEL(C, F, T)    (c ? T : F)
#endif

#define ADR(bVAR) (*bVAR)

#if defined(ALLOW_TC_COMPARE)
    #define LE(a,b)     (a <= b)
    #define LT(a,b)     (a < b)
    #define EQ(a,b)     (a == b)
    #define NE(a,b)     (a != b)
    #define GT(a,b)     (a > b)
    #define GE(a,b)     (a >= b)
    #define AND         &&
    #define AND_THEN    ) if(
    #define OR          ||
    #define NOT         !
#endif

#if defined(ALLOW_TC_MATH)
    #define ADD(a,b)    (a + b)
    #define SUB(a,b)    (a - b)
    #define MUL(a,b)    (a * b)
    #define DIV(a,b)    (a / b)
    #define EXPT(a,b)   (pow(a, b))
    #define MOD         %
    
    template<class T>
    static T ABS(T value) { return pow(pow(value, 2), 0.5); }
#endif

#if defined(ALLOW_TC_BITWISE)
    // Bitwise shift left
    #define SHL(n, b)   (n << b)
    // Bitwise shift right
    #define SHR(n, b)   (n >> b)
    // Bitwise Exclusive-or
    #define XOR         ^
    // Complement (bitwise not)
    #define COMP(n)     (~n)
#endif


/**
 * @brief   Rising edge trigger. Takes boolean input signal, CLK,
 *          in function call. The output, Q, will be TRUE for a single
 *          evaluation when CLK goes from FALSE to TRUE. 
 *          
 *          Q will return to false on the next subsequent call.
 */
class R_TRIG
{
public:
    R_TRIG() { CLK = false; };

    R_TRIG& operator()(bool in) 
    { 
        lastState = CLK; 
        CLK = in;

        IF CLK AND NOT(CLK == lastState) THEN
            Q = true;
        ELSE
            Q = false;
        END_IF

        return *this;
    }

    bool Q;

private:
    bool CLK;

    bool lastState;

};

/**
 * @brief   Falling edge trigger. Takes boolean input signal, CLK,
 *          in function call. The output, Q, will be TRUE for a single
 *          evaluation when CLK goes from TRUE to FALSE. 
 * 
 *          Q will return to false on the next subsequent call.
 */
class F_TRIG
{
public:
    F_TRIG() { state = false; };

    F_TRIG& operator()(bool CLK) 
    { 
        lastState = state; 
        state = CLK;

        IF NOT state AND NOT(state == lastState) THEN
            Q = true;
        ELSE
            Q = false;
        END_IF

        return *this;
    }

    bool Q;

private:
    bool state;

    bool lastState;

};


#endif // TC3_SYNTAX_H
/**
 * @file square_recog.hpp
 * @brief Class to identify the square on the center of a corridor (needed to cut the walls into trapeziums)
 * @author Igor de Carvalho Coelho
 */

#ifndef SQUARE_RECOG_HPP
#define SQUARE_RECOG_HPP

#include "general.hpp"

class SquareRecog{
    public:
        cv::Rect findBiggestSquare(const cv::Mat &src);
    private:
        SquareRecog(){}
        ~SquareRecog(){}
};


#endif

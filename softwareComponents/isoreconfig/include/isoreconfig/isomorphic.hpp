#include <configuration/rofibot.hpp>

using Matrix = arma::mat44;
using Positions = std::vector< Matrix >;
using namespace rofi::configuration;

Positions decomposeRofibot( Rofibot rb );

#include <configuration/rofibot.hpp>

std::vector< arma::mat44 > decomposeUniversalModule( const rofi::configuration::Module& mod, const arma::mat44& modPos );
std::vector< arma::mat44 > decomposeRofibot( const rofi::configuration::Rofibot& rb );
arma::vec4 getCenter( const rofi::configuration::Rofibot& rb );
arma::vec4 getCenter( const std::vector< arma::mat44 >& points );

#include <isoreconfig/isomorphic.hpp>
#include <cassert>

using Vector = arma::vec4;
using Matrix = arma::mat44;
using Points = std::vector< Matrix >;
using namespace rofi::configuration;

// Assumes <mod> is a UniversalModule
Points decomposeUniversalModule( const Module& mod, const Matrix& modPos )
{
    assert( mod.type == ModuleType::Universal );

    std::span<const Component> comps = mod.components();
    std::vector< Matrix > result;

    for ( int compIndex = 0; compIndex < 6; ++compIndex )
    {
        assert( comps[ compIndex ].type == ComponentType::Roficom );

        result.push_back( modPos * mod.getComponentRelativePosition( compIndex ) * matrices::translate( { -0.5, 0, 0 } ) );
    } 

    return result;
}

// Assumes the rofibot has been prepared
// Assumes the rofibot consists of only UniversalModules
Points decomposeRofibot( const Rofibot& rb )
{
    auto [ ok, err ] = rb.isValid();
    assert( ok );

    Points result;

    for ( const /*Rofibot::ModuleInfo*/ auto& modInf : rb.modules() )
        for ( const Matrix& pos : decomposeUniversalModule( *modInf.module, *modInf.absPosition ) )
            result.push_back( pos );

    return result;
}

Vector getCenter( const Rofibot& rb )
{
    return getCenter( decomposeRofibot( rb ) );
}

Vector getCenter( const Points& points )
{
    Vector result = { 0, 0, 0, 0 };
    
    for ( const Matrix& point : points )
        result += point.col( 3 );

    assert( result( 3 ) >= 0 && size_t( result( 3 ) ) == points.size() );

    result( 3 ) = 1;
    return result / Vector( { points.size(), points.size(), points.size(), points.size() } );
}


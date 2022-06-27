#include <configuration/rofibot.hpp>
#include <cassert>

#include <iostream>

using Matrix = arma::mat44;
using Positions = std::vector< Matrix >;
using namespace rofi::configuration;

/* Positions decomposeModule( Module& mod )
{
    std::vector< Matrix > result;

    for ( const Component& comp : mod.components() )
    {
        result.push_back( mod.getComponentPosition( mod.componentIdx( comp )));
    }

    return result;
} */

Positions decomposeRofibot( Rofibot rb )
{
    rb.prepare();
    auto [ ok, err ] = rb.isValid();
    assert( ok );

    Positions result;

    for ( const /*Rofibot::ModuleInfo*/ auto& modInf : rb.modules() )
    {
        for ( const Component& comp : modInf.module->components() )
        {
            result.push_back( comp.getPosition() );
        }
    }

    return result;
}

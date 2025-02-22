#include <catch2/catch.hpp>

#include <configuration/rofibot.hpp>
#include <configuration/universalModule.hpp>
#include <configuration/unknownModule.hpp>

namespace {

using namespace rofi::configuration;
using namespace rofi::configuration::roficom;
using namespace rofi::configuration::matrices;

TEST_CASE( "UnknownModule (base Module) Test" ) {
    auto m = UnknownModule( { Component{ ComponentType::Roficom, {}, {}, nullptr } }, 1, {}, 42 );
    CHECK( m.bodies().size() == 0 );
    CHECK( m.components().size() == 1 );
    CHECK( m.connectors().size() == 1 );
    CHECK( m.getId() == 42 );
    CHECK( m.setId( 66 ) );
    CHECK( m.getId() == 66 );
}

TEST_CASE( "Universal Module Test" ) {
    SECTION( "Creation" ) {
        auto um = UniversalModule( 0, 0_deg, 0_deg, 0_deg );
        CHECK( um.components().size() == 10 );
        um.prepare();
        REQUIRE( um.getOccupiedRelativePositions().size() == 2 );
        CHECK( equals( um.getOccupiedRelativePositions()[ 0 ], identity ) );
        CHECK( equals( center( um.getOccupiedRelativePositions()[ 1 ] ), { 0, 0, 1, 1 } ) );
    }

    SECTION( "roficomConnections" ) {
        auto um = UniversalModule( 0, 0_deg, 0_deg, 0_deg );
        CHECK( um.connectors().size() == 6 );

        for ( size_t i = 0; i < um.connectors().size(); i++ ) {
            INFO( "Connector number: " << i );
            CHECK( um.connectors()[ i ].type == ComponentType::Roficom );
        }

        REQUIRE( um.components().size() >= 6 );
        for ( size_t i = 0; i < um.components().size(); i++ ) {
            INFO( "Number of connectors: " << um.connectors().size() );
            INFO( "Component number: " << i );
            if ( i < um.connectors().size() ) {
                CHECK( um.components()[ i ].type == ComponentType::Roficom );
            } else {
                CHECK( um.components()[ i ].type != ComponentType::Roficom );
            }
        }
    }

    SECTION( "Position - default" ) {
        auto um = UniversalModule( 0, 0_deg, 0_deg, 0_deg );
        // A part
        CHECK( equals( um.getComponentRelativePosition( 0 ), identity ) );
        CHECK( equals( um.getComponentRelativePosition( 1 ), rotate( M_PI, { 0, 0, 1 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
        CHECK( equals( um.getComponentRelativePosition( 2 ), rotate( - M_PI_2, { 0, 1, 0 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
        CHECK( equals( um.getComponentRelativePosition( 6 ), identity ) );
        CHECK( equals( um.getComponentRelativePosition( 7 ), identity ) );
        // B part
        // -X
        CHECK( equals( um.getComponentRelativePosition( 3 ), translate( { 0, 0, 1 } ) * rotate( M_PI, { 0, 0, 1 } )
                                                   * rotate( M_PI, { 1, 0, 0 } ) ) );
        // +X
        CHECK( equals( um.getComponentRelativePosition( 4 ), translate( { 0, 0, 1 } ) * rotate( M_PI, { 0, 1, 0 } )
                                                   * rotate( M_PI, { 0, 0, 1 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
        // -Z
        CHECK( equals( um.getComponentRelativePosition( 5 ), translate( { 0, 0, 1 } ) * rotate( M_PI, { 0, 1, 0 } )
                                                   * rotate( - M_PI_2, { 0, 1, 0 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
        // Body
        CHECK( equals( um.getComponentRelativePosition( 8 ), translate( { 0, 0, 1 } ) * rotate( M_PI, { 0, 1, 0 } ) ) );
        // Shoe
        CHECK( equals( um.getComponentRelativePosition( 9 ), { { -1, 0,  0, 0 }
                                                     , {  0, 1,  0, 0 }
                                                     , {  0, 0, -1, 1 }
                                                     , {  0, 0,  0, 1 } } ) );
    }

    SECTION( "Position - rotated gamma" ) {
        SECTION( "degrees" ) {
            auto um = UniversalModule( 0, 0_deg, 0_deg, 90_deg );
            // A part
            CHECK( equals( um.getComponentRelativePosition( 0 ), identity ) );
            CHECK( equals( um.getComponentRelativePosition( 1 ), rotate( M_PI, { 0, 0, 1 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
            CHECK( equals( um.getComponentRelativePosition( 2 ), rotate( - M_PI_2, { 0, 1, 0 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
            CHECK( equals( um.getComponentRelativePosition( 6 ), identity ) );
            CHECK( equals( um.getComponentRelativePosition( 7 ), identity ) );
            // B part
            // -X
            CHECK( equals( um.getComponentRelativePosition( 3 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                        * rotate( M_PI, { 0, 0, 1 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
            // +X
            CHECK( equals( um.getComponentRelativePosition( 4 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                    * rotate( M_PI, { 0, 1, 0 } ) * rotate( M_PI, { 0, 0, 1 } )
                                                    * rotate( M_PI, { 1, 0, 0 } ) ) );
            // -Z
            CHECK( equals( um.getComponentRelativePosition( 5 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                        * rotate( M_PI, { 0, 1, 0 } ) * rotate( - M_PI_2, { 0, 1, 0 } )
                                                        * rotate( M_PI, { 1, 0, 0 } ) ) );
            // Body
            CHECK( equals( um.getComponentRelativePosition( 8 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                    * rotate( M_PI, { 0, 1, 0 } ) ) );
            // Shoe
            CHECK( equals( um.getComponentRelativePosition( 9 ), { {  0, -1,  0, 0 } // shoeB
                                                        , { -1,  0,  0, 0 }
                                                        , {  0,  0, -1, 1 }
                                                        , {  0,  0,  0, 1 } } ) );
        }
    }

    SECTION( "radians" ) {
        auto um = UniversalModule( 0, 0_rad, 0_rad, 1.57079632679489661923_rad ); // 0, 0, M_PI_2
        // A part
        CHECK( equals( um.getComponentRelativePosition( 0 ), identity ) );
        CHECK( equals( um.getComponentRelativePosition( 1 ), rotate( M_PI, { 0, 0, 1 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
        CHECK( equals( um.getComponentRelativePosition( 2 ), rotate( - M_PI_2, { 0, 1, 0 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
        CHECK( equals( um.getComponentRelativePosition( 6 ), identity ) );
        CHECK( equals( um.getComponentRelativePosition( 7 ), identity ) );
        // B part
        // -X
        CHECK( equals( um.getComponentRelativePosition( 3 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                    * rotate( M_PI, { 0, 0, 1 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
        // +X
        CHECK( equals( um.getComponentRelativePosition( 4 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                   * rotate( M_PI, { 0, 1, 0 } ) * rotate( M_PI, { 0, 0, 1 } )
                                                   * rotate( M_PI, { 1, 0, 0 } ) ) );
        // -Z
        CHECK( equals( um.getComponentRelativePosition( 5 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                    * rotate( M_PI, { 0, 1, 0 } ) * rotate( - M_PI_2, { 0, 1, 0 } )
                                                    * rotate( M_PI, { 1, 0, 0 } ) ) );
        // Body
        CHECK( equals( um.getComponentRelativePosition( 8 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                   * rotate( M_PI, { 0, 1, 0 } ) ) );
        // Shoe
        CHECK( equals( um.getComponentRelativePosition( 9 ), { {  0, -1,  0, 0 } // shoeB
                                                     , { -1,  0,  0, 0 }
                                                     , {  0,  0, -1, 1 }
                                                     , {  0,  0,  0, 1 } } ) );
    }

    SECTION( "Position - rotated beta + gamma" ) {
        SECTION( "degrees" ) {
            auto um = UniversalModule( 0, 0_deg, Angle::deg( -90 ), 90_deg );
            // A part
            CHECK( equals( um.getComponentRelativePosition( 0 ), identity ) );
            CHECK( equals( um.getComponentRelativePosition( 1 ), rotate( M_PI, { 0, 0, 1 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
            CHECK( equals( um.getComponentRelativePosition( 2 ), rotate( - M_PI_2, { 0, 1, 0 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
            CHECK( equals( um.getComponentRelativePosition( 6 ), identity ) );
            CHECK( equals( um.getComponentRelativePosition( 7 ), identity ) );
            // B part
            // -X
            CHECK( equals( um.getComponentRelativePosition( 3 ), rotate( M_PI_2, { 0, 0, 1 } )
                                                        * translate( { 0, 0, 1 } )
                                                        * rotate( M_PI, { 0, 1, 0 } )
                                                        * rotate( - M_PI_2, { 1, 0, 0 } ) ) );
            // +X
            CHECK( equals( um.getComponentRelativePosition( 4 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                        * rotate( M_PI, { 0, 1, 0 } )  * rotate( - M_PI_2, { 1, 0, 0 } )
                                                        * rotate( M_PI, { 0, 0, 1 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
            // -Z
            CHECK( equals( um.getComponentRelativePosition( 5 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                        * rotate( M_PI, { 0, 1, 0 } )  * rotate( - M_PI_2, { 1, 0, 0 } )
                                                        * rotate( - M_PI_2, { 0, 1, 0 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
            // Body
            CHECK( equals( um.getComponentRelativePosition( 8 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                        * rotate( M_PI, { 0, 1, 0 } ) ) );
            // Shoe
            CHECK( equals( um.getComponentRelativePosition( 9 ), { {  0, 0, -1, 0 }
                                                        , { -1, 0,  0, 0 }
                                                        , {  0, 1,  0, 1 }
                                                        , {  0, 0,  0, 1 } } ) );
        }

        SECTION( "radians" ) {
            auto um = UniversalModule( 0, 0_rad, Angle::rad( - Angle::pi / 2 ), Angle::rad( Angle::pi / 2 ) );
            // A part
            CHECK( equals( um.getComponentRelativePosition( 0 ), identity ) );
            CHECK( equals( um.getComponentRelativePosition( 1 ), rotate( M_PI, { 0, 0, 1 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
            CHECK( equals( um.getComponentRelativePosition( 2 ), rotate( - M_PI_2, { 0, 1, 0 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
            CHECK( equals( um.getComponentRelativePosition( 6 ), identity ) );
            CHECK( equals( um.getComponentRelativePosition( 7 ), identity ) );
            // B part
            // -X
            CHECK( equals( um.getComponentRelativePosition( 3 ), rotate( M_PI_2, { 0, 0, 1 } )
                                                        * translate( { 0, 0, 1 } )
                                                        * rotate( M_PI, { 0, 1, 0 } )
                                                        * rotate( - M_PI_2, { 1, 0, 0 } ) ) );
            // +X
            CHECK( equals( um.getComponentRelativePosition( 4 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                        * rotate( M_PI, { 0, 1, 0 } )  * rotate( - M_PI_2, { 1, 0, 0 } )
                                                        * rotate( M_PI, { 0, 0, 1 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
            // -Z
            CHECK( equals( um.getComponentRelativePosition( 5 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                        * rotate( M_PI, { 0, 1, 0 } )  * rotate( - M_PI_2, { 1, 0, 0 } )
                                                        * rotate( - M_PI_2, { 0, 1, 0 } ) * rotate( M_PI, { 1, 0, 0 } ) ) );
            // Body
            CHECK( equals( um.getComponentRelativePosition( 8 ), rotate( M_PI_2, { 0, 0, 1 } ) * translate( { 0, 0, 1 } )
                                                        * rotate( M_PI, { 0, 1, 0 } ) ) );
            // Shoe
            CHECK( equals( um.getComponentRelativePosition( 9 ), { {  0, 0, -1, 0 }
                                                        , { -1, 0,  0, 0 }
                                                        , {  0, 1,  0, 1 }
                                                        , {  0, 0,  0, 1 } } ) );
        }
    }

    SECTION( "Connector translations" ) {
        CHECK( UniversalModule::translateComponent( "A-X" ) == 0 );
        CHECK( UniversalModule::translateComponent( "A+X" ) == 1 );
        CHECK( UniversalModule::translateComponent( "A-Z" ) == 2 );
        CHECK( UniversalModule::translateComponent( "B-X" ) == 3 );
        CHECK( UniversalModule::translateComponent( "B+X" ) == 4 );
        CHECK( UniversalModule::translateComponent( "B-Z" ) == 5 );
    }
}

TEST_CASE( "Two modules next to each other" ) {
    ModuleId idCounter = 0;
    Rofibot bot;
    auto& m1 = bot.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    auto& m2 = bot.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    auto con = m2.connectors()[ 1 ];
    connect( m1.connectors()[ 0 ], con, Orientation::South );
    connect< RigidJoint >( m1.bodies()[ 0 ], { 0, 0, 0 }, identity );
    REQUIRE_NOTHROW( bot.prepare() );

    SECTION( "The second is just moved to left by one" ) {
        Matrix new_origin = identity * translate( { -1, 0, 0 } );
        REQUIRE( !equals( identity, new_origin ) );
        Matrix mat = identity;
        CHECK( equals( center( bot.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 0 ) )
                     , center( new_origin ) ) );

        mat = new_origin * rotate( M_PI, { 0, 0, 1 } );
        CHECK( equals( center( bot.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 1 ) ), center( mat ) ) );

        mat = new_origin * rotate( - M_PI_2, { 0, 1, 0 } );
        CHECK( equals( center( bot.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 2 ) ), center( mat ) ) );
        CHECK( equals( center( bot.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 6 ) ), center( new_origin ) ) );
        CHECK( equals( center( bot.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 7 ) ), center( new_origin ) ) );

        mat =  new_origin * m1.getComponentRelativePosition( 3 );
        CHECK( equals( center( bot.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 3 ) ), center( mat ) ) );

        mat = new_origin * m1.getComponentRelativePosition( 4 );
        CHECK( equals( center( bot.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 4 ) ), center( mat ) ) );

        mat = new_origin * m1.getComponentRelativePosition( 5 );
        CHECK( equals( center( bot.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 5 ) ), center( mat ) ) );

        mat = new_origin * m1.getComponentRelativePosition( 8 );
        CHECK( equals( center( bot.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 8 ) ), center( mat ) ) );

        mat = { { -1, 0, 0, -1 }, { 0, 1, 0, 0 }, { 0, 0, -1, 1 }, { 0, 0, 0, 1 } };
        CHECK( equals( center( bot.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 9 ) ), center( mat ) ) );

        mat = new_origin * m1.getComponentRelativePosition( 9 );
        CHECK( equals( center( bot.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 9 ) ), center( mat ) ) );
    }
}

TEST_CASE( "Two modules - different angles" ) {
    int idCounter = 0;
    Rofibot bot;
    auto& m1 = bot.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    auto& m2 = bot.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    connect( m1.connectors()[ 3 ], m2.connectors()[ 0 ], Orientation::South );
    connect< RigidJoint >( m1.bodies()[ 0 ], { 0, 0, 0 }, identity );
    REQUIRE_NOTHROW( bot.prepare() );

    SECTION( "BodyA " ) {
        CHECK( equals( bot.getModulePosition( m1.getId() ), identity ) );
        CHECK( equals( center( bot.getModulePosition( m2.getId() ) ), center( identity * translate( { 1, 0, 1 } ) ) ) );
    }

    SECTION( "BodyB" ) {
    Matrix m1shoeB = m1.getComponentRelativePosition( 9 );
    Matrix m2shoeB = bot.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 9 );
    CHECK( equals( m1shoeB, { { -1, 0,  0, 0 }
                            , {  0, 1,  0, 0 }
                            , {  0, 0, -1, 1 }
                            , {  0, 0,  0, 1 } } ) );
    CHECK( equals( center( m2shoeB ), center( translate( { 1, 0, 2 } ) ) ) );
    }
}

TEST_CASE( "Three modules -- connect docks 3 to 0s " ) {
    int idCounter = 0;
    Rofibot bot;
    auto& m1 = bot.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    auto& m2 = bot.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    auto& m3 = bot.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    connect( m1.connectors()[ 3 ], m2.connectors()[ 0 ], Orientation::South );
    connect( m2.connectors()[ 3 ], m3.connectors()[ 0 ], Orientation::South );
    connect< RigidJoint >( m1.bodies()[ 0 ], { 0, 0, 0 }, identity );
    REQUIRE_NOTHROW( bot.prepare() );

    SECTION( "Modules are well placed" ) {
        CHECK( equals( center( bot.getModulePosition( m1.getId() ) ), center( identity ) ) );
        CHECK( equals( center( bot.getModulePosition( m2.getId() ) ), center( translate( { 1, 0, 1 } ) ) ) );
        CHECK( equals( center( bot.getModulePosition( m3.getId() ) ), center( translate( { 2, 0, 2 } ) ) ) );
    }

    SECTION( "Shoes A" ) {
        CHECK( equals( bot.getModulePosition( m1.getId() ) * m1.getComponentRelativePosition( 6 ), identity ) );
        CHECK( equals( bot.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 6 ), translate( { 1, 0, 1 } ) ) );
        CHECK( equals( bot.getModulePosition( m3.getId() ) * m3.getComponentRelativePosition( 6 ), translate( { 2, 0, 2 } ) ) );
    }

    SECTION( "Shoes B" ) {
        Matrix x = bot.getModulePosition( m1.getId() ) * m1.getComponentRelativePosition( 9 );
        CHECK( equals( bot.getModulePosition( m1.getId() ) * m1.getComponentRelativePosition( 9 )
                     , translate( { 0, 0, 1 } ) * rotate( M_PI, { 0, 1, 0 } ) ) );
        CHECK( equals( bot.getModulePosition( m2.getId() ) * m2.getComponentRelativePosition( 9 )
                     , translate( { 1, 0, 2 } ) * rotate( M_PI, { 0, 1, 0 } ) ) );
        CHECK( equals( bot.getModulePosition( m3.getId() ) * m3.getComponentRelativePosition( 9 )
                     , translate( { 2, 0, 3 } ) * rotate( M_PI, { 0, 1, 0 } ) ) );
    }
}

TEST_CASE( "Basic rofibot manipulation" ) {
    using namespace rofi;
    int idCounter = 0;
    Rofibot bot;
    auto& m1 = bot.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    CHECK( m1.getId() == 0 );
    CHECK( bot.getModule( 0 )->getId() == 0 );
    REQUIRE( m1.getId() == 0 );
    REQUIRE( m1.parent == &bot );
    CHECK( &m1 == bot.getModule( 0 ) );
    auto& m2 = bot.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    REQUIRE( m2.getId() == 1 );
    CHECK( m1.getId() == 0 );
    CHECK( bot.getModule( 0 )->getId() == 0 );
    REQUIRE( &m1 == bot.getModule( 0 ) );
    auto& m3 = bot.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    REQUIRE( m3.getId() == 2 );
    REQUIRE( m1.parent == &bot );
    auto& m4 = bot.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    REQUIRE( m4.getId() == 3 );
    auto& m5 = bot.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    REQUIRE( m5.getId() == 4 );
    CHECK( bot.modules().size() == 5 );

    CHECK( bot.roficomConnections().size() == 0 );
    connect( m1.connectors()[ 5 ], m2.connectors()[ 2 ], Orientation::North );
    connect( m2.connectors()[ 5 ], m3.connectors()[ 2 ], Orientation::North );
    connect( m3.connectors()[ 5 ], m4.connectors()[ 2 ], Orientation::North );
    connect( m4.connectors()[ 5 ], m5.connectors()[ 2 ], Orientation::North );
    CHECK( bot.roficomConnections().size() == 4 );
    connect< RigidJoint >( m1.bodies()[ 0 ], { 0, 0, 0 }, identity );
    m1.setGamma( Angle::deg( 90 ) );
    auto [ b, str ] = bot.isValid();
    CHECK( !b ); // because the configuration is not prepared
    auto [ b2, str2 ] = bot.validate();
    CHECK( b2 );
    if ( !b2 )
        std::cout << "Error: " << str2 << "\n";
}

TEST_CASE( "Colliding configuration" ) {
    using namespace rofi;
    int idCounter = 0;
    Rofibot bot;
    auto& m1 = bot.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    auto& m2 = bot.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    auto& m3 = bot.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    auto& m4 = bot.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    auto& m5 = bot.insert( UniversalModule( idCounter++, 0_deg, 0_deg, 0_deg ) );
    CHECK( bot.modules().size() == 5 );

    connect( m1.connectors()[ 1 ], m2.connectors()[ 2 ], Orientation::North );
    connect( m2.connectors()[ 1 ], m3.connectors()[ 2 ], Orientation::North );
    connect( m3.connectors()[ 1 ], m4.connectors()[ 2 ], Orientation::North );
    connect( m4.connectors()[ 1 ], m5.connectors()[ 2 ], Orientation::North );
    CHECK( bot.roficomConnections().size() == 4 );
    connect< RigidJoint >( m1.bodies()[ 0 ], { 0, 0, 0 }, identity );
    auto [ b, str ] = bot.validate();
    CHECK( !b );
}

TEST_CASE( "Changing modules ID" ) {
    using namespace rofi;
    Rofibot bot;

    auto& m1 = bot.insert( UniversalModule( 0, 0_deg, 0_deg, 0_deg ) );
    auto& m2 = bot.insert( UniversalModule( 1, 0_deg, 0_deg, 0_deg ) );
    auto& m3 = bot.insert( UniversalModule( 2, 0_deg, 0_deg, 0_deg ) );
    connect( m1.connectors()[ 5 ], m2.connectors()[ 2 ], Orientation::North );
    connect( m2.connectors()[ 5 ], m3.connectors()[ 2 ], Orientation::North );
    connect< RigidJoint >( m1.bodies()[ 0 ], { 0, 0, 0 }, identity );

    CHECK( bot.validate().first );

    CHECK( m1.getId() == 0 );
    CHECK( m2.getId() == 1 );
    CHECK( m3.getId() == 2 );

    CHECK( m2.setId( 42 ) );
    CHECK( bot.validate().first );
    CHECK( m1.getId() == 0 );
    CHECK( m2.getId() != 1 );
    CHECK( m2.getId() == 42 );
    CHECK( m3.getId() == 2 );

    CHECK( !m2.setId( 0 ) );
    CHECK( m1.getId() == 0 );
    CHECK( m2.getId() == 42 );
    CHECK( m3.getId() == 2 );

    CHECK( m1.setId( 66 ) );
    CHECK( m3.setId( 78 ) );

    CHECK( m1.getId() == 66 );
    CHECK( m2.getId() == 42 );
    CHECK( m3.getId() == 78 );
}

TEST_CASE( "Configurable joints" ) {
    SECTION( "default" ) {
        auto m = UniversalModule( 42, 0_deg, 0_rad, 0_deg );
        int i = 0;
        for ( auto& j : m.configurableJoints() ) {
            static_assert( std::is_same_v< decltype( j ), Joint & > );
            i++;
        }
        REQUIRE( i == 3 );
    }
    SECTION( "const" ) {
        auto m = UniversalModule( 42, 0_deg, 0_rad, 0_deg );
        int i = 0;
        for ( auto& j : std::as_const( m ).configurableJoints() ) {
            static_assert( std::is_same_v< decltype( j ), const Joint & > );
            i++;
        }
        CHECK( i == 3 );
    }
    SECTION( "equality" ) {
        auto m = UniversalModule( 42, 0_deg, 0_rad, 0_deg );

        auto joints = m.configurableJoints();
        auto cjoints = std::as_const( m ).configurableJoints();

        auto it = joints.begin();
        auto cit = cjoints.begin();
        while ( it != joints.end() && cit != cjoints.end() ) {
            CHECK( &*it == &*cit ); // Check address
            ++it;
            ++cit;
        }
        CHECK( it == joints.end() );
        CHECK( cit == cjoints.end() );
    }
}

TEST_CASE( "Connect and disconnect" ) {
    Rofibot bot;

    SECTION( "Disconnect roficomConnections disconnects two modules" ) {
        auto& m1 = bot.insert( UniversalModule( 42, 0_deg, 0_deg, 0_deg ) );
        auto& m2 = bot.insert( UniversalModule( 66, 0_deg, 0_deg, 0_deg ) );

        auto h = connect< RigidJoint >( m1.getConnector( "A-Z" ), { 0, 0, 0 }, identity );

        CHECK( bot.roficomConnections().empty() );
        auto j = connect( m1.getConnector( "A+X" ), m2.getConnector( "B-Z" ), roficom::Orientation::North );
        CHECK( bot.roficomConnections().size() == 1 );
        bot.prepare();
        CHECK( bot.isPrepared() );
        bot.disconnect( j );
        CHECK( !bot.isPrepared() );
        CHECK( bot.roficomConnections().empty() );
    }

    SECTION( "Disconnect on spaceJoint" ) {
        auto& m = bot.insert( UniversalModule( 42, 0_deg, 0_deg, 0_deg ) );

        CHECK( bot.referencePoints().empty() );
        auto h = connect< RigidJoint >( m.getConnector( "A-Z" ), { 0, 0, 0 }, identity );
        CHECK( !bot.referencePoints().empty() );

        bot.prepare();
        CHECK( bot.isPrepared() );
        bot.disconnect( h );
        CHECK( !bot.isPrepared() );
        CHECK( bot.referencePoints().empty() );
    }

    SECTION( "Reconnect - simple" )
    {
        auto & m1 = static_cast< UniversalModule & >(
                bot.insert( UniversalModule( 42, 90_deg, 0_deg, 0_deg ) ) );
        auto & m2 = static_cast< UniversalModule & >(
                bot.insert( UniversalModule( 66, 0_deg, 90_deg, 0_deg ) ) );

        auto h = connect< RigidJoint >( m1.getConnector( "A-Z" ), { 0, 0, 0 }, identity );

        auto j1 = connect( m1.getConnector( "A+X" ),
                           m2.getConnector( "A-X" ),
                           roficom::Orientation::South );

        bot.prepare();
        REQUIRE( bot.isPrepared() );
        CHECK( bot.isValid().first );

        bot.disconnect( j1 );

        CHECK_THROWS( bot.prepare() );
    }

    SECTION( "Reconnect" )
    {
        auto & m1 = static_cast< UniversalModule & >(
                bot.insert( UniversalModule( 42, 90_deg, 0_deg, 0_deg ) ) );
        auto & m2 = static_cast< UniversalModule & >(
                bot.insert( UniversalModule( 66, 0_deg, 90_deg, 0_deg ) ) );

        auto h = connect< RigidJoint >( m1.getConnector( "A-Z" ), { 0, 0, 0 }, identity );

        auto j1 = connect( m1.getConnector( "A+X" ),
                           m2.getConnector( "A-X" ),
                           roficom::Orientation::South );

        bot.prepare();
        REQUIRE( bot.isPrepared() );
        CHECK( bot.isValid().first );

        m1.setAlpha( 0_deg );
        m2.setBeta( 0_deg );
        auto j2 = connect( m1.getConnector( "B-X" ),
                           m2.getConnector( "B+X" ),
                           roficom::Orientation::South );

        bot.prepare();
        REQUIRE( bot.isPrepared() );
        CHECK( bot.isValid().first );

        bot.disconnect( j1 );

        bot.prepare();
        REQUIRE( bot.isPrepared() );
        CHECK( bot.isValid().first );

        m1.setAlpha( 90_deg );
        m2.setBeta( 90_deg );

        bot.prepare();
        REQUIRE( bot.isPrepared() );
        CHECK( bot.isValid().first );
    }
}

TEST_CASE( "Get near connector" ) {
    Rofibot bot;

    SECTION( "two straight modules" ) {
        auto& m1 = bot.insert( UniversalModule( 42, 0_deg, 0_deg, 0_deg ) );
        auto& m2 = bot.insert( UniversalModule( 66, 0_deg, 0_deg, 0_deg ) );

        auto h = connect< RigidJoint >( m1.getConnector( "A-Z" ), { 0, 0, 0 }, identity );

        connect( m1.getConnector( "A+X" ), m2.getConnector( "A+X" ), roficom::Orientation::North );

        bot.prepare();
        CHECK( bot.isValid().first );

        SECTION( "Works with fixed connection" ) {
            connect( m1.getConnector( "B-X" ), m2.getConnector( "B-X" ), roficom::Orientation::North );

            bot.prepare();
            CHECK( bot.isValid().first );
        }

        SECTION( "Get near connector" ) {
            auto nearConnector = m1.getConnector( "B-X" ).getNearConnector();
            REQUIRE( nearConnector.has_value() );
            CHECK( nearConnector->first == m2.getConnector( "B-X" ) );
            CHECK( nearConnector->second == roficom::Orientation::North );

            connect( m1.getConnector( "B-X" ), nearConnector->first, nearConnector->second );

            bot.prepare();
            CHECK( bot.isValid().first );
        }
    }

    SECTION( "two straight modules - throws if not prepared in advance" ) {
        auto& m1 = bot.insert( UniversalModule( 42, 0_deg, 0_deg, 0_deg ) );
        auto& m2 = bot.insert( UniversalModule( 66, 0_deg, 0_deg, 0_deg ) );

        auto h = connect< RigidJoint >( m1.getConnector( "A-Z" ), { 0, 0, 0 }, identity );

        connect( m1.getConnector( "A+X" ), m2.getConnector( "A+X" ), roficom::Orientation::North );

        REQUIRE_FALSE( bot.isPrepared() );

        CHECK_THROWS( m1.getConnector( "B-X" ).getNearConnector() );
    }
}

} // namespace

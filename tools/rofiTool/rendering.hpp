#pragma once
#include <configuration/rofibot.hpp>
#include <isoreconfig/isomorphic.hpp>

void renderConfiguration( rofi::configuration::Rofibot configuration, const std::string& configName = "unknown" );
void renderPoints( rofi::configuration::Rofibot configuration, const std::string& configName = "unknown" );

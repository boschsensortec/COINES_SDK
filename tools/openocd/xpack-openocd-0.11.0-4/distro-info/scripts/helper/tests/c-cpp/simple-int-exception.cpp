/* ----------------------------------------------------------------------------
 * This file is part of the xPack distribution.
 *   (https://xpack.github.io)
 * Copyright (c) 2020 Liviu Ionescu.
 *
 * Permission to use, copy, modify, and/or distribute this software 
 * for any purpose is hereby granted, under the terms of the MIT license.
 * -------------------------------------------------------------------------- */

#include <iostream>
#include <exception>
 
void
func(void)
{
  throw 42;
}

int
main(int argc, char* argv[])
{
  try {
    func();
  } catch(int val) {
    std::cout << val << std::endl;
  } catch(std::exception& e) {
    std::cout << "Other" << std::endl;
  } 

  return 0; 
}

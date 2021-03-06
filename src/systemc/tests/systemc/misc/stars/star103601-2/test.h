/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/

/*****************************************************************************

  test.h -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

struct test : sc_module {
  
  sc_in<bool> reset;
  sc_in_clk clk;
  sc_in<sc_uint<8> >  dati;
  sc_out<sc_uint<8> >  dato;
  sc_out<bool> done;

  SC_HAS_PROCESS( test );
  
  test (const char *NAME) : sc_module(NAME) {
    SC_CTHREAD( reset_loop, clk.pos() );
    reset_signal_is(reset,true);
    end_module();
  }
  
  void reset_loop();
};


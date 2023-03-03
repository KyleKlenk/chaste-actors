
/*
Copyright (c) 2005-2019, University of Oxford.
All rights reserved.

University of Oxford means the Chancellor, Masters and Scholars of the
University of Oxford, having an administrative office at Wellington
Square, Oxford OX1 2JD, UK.

This file is part of Chaste.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.
* Neither the name of the University of Oxford nor the names of its
contributors may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

/*
*  Use this code to test S1-S2 protocal 
*
*/
#ifndef TESTCHASTES1S2BIDOMAIN_HPP_
#define TESTCHASTES1S2BIDOMAIN_HPP_

/*
* HOW_TO_TAG Cardiac/Problem definition
* Set up and run basic bidomain simulations
*/

/*
* = An example showing how to run bidomain simulations =
*
* == Introduction ==
*
* In this tutorial we show how Chaste is used to run a standard bidomain simulation.
* Note that monodomain simulations are run very similarly.
*
* The first thing that needs to be done, when writing any Chaste test,
* is to include the following header.
*/

#include <cxxtest/TestSuite.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <vector>
#include <cmath>
#include "CellMLToSharedLibraryConverter.hpp"
#include "DynamicCellModelLoader.hpp"
#include "LuoRudy1991.hpp"
#include "AbstractCardiacCell.hpp"
#include "AbstractCardiacCellFactory.hpp"
#include "BidomainProblem.hpp"
#include "SimpleStimulus.hpp"
#include "PetscSetupAndFinalize.hpp"
#include "GrayPathmanathan2016Rabbit.hpp"
#include "EulerIvpOdeSolver.hpp"
#include "ZeroStimulusCellFactory.hpp"
#include "TetrahedralMesh.hpp"
#include "PetscTools.hpp"


//template<unsigned DIM>
class PointStimulus3dCellFactory : public AbstractCardiacCellFactory<3> {

  /*Define the stimulus */
  /* Simple Stimulus */
  private:
    boost::shared_ptr<SimpleStimulus> mpStimulus;

  public:
    // ** Constructor **
    PointStimulus3dCellFactory() : AbstractCardiacCellFactory<3>(), 
      mpStimulus( new SimpleStimulus(-214500, 2.0,0.0)) { }

    // ** Methods **
    AbstractCardiacCell* CreateCardiacCellForTissueNode(Node<3>* pNode) {
      double x = pNode->rGetLocation()[0];
      double y = pNode->rGetLocation()[1];
      double z = pNode->rGetLocation()[2];
      boost::shared_ptr<EulerIvpOdeSolver> p_solver(new EulerIvpOdeSolver());

      if ((0.1333<=x) && (x<=0.1333+0.05) && (0.020833<=y)&&(y<=0.020833+0.01) && (0.0<=z)&&(z<=0.0+0.0025)) {
        /* Create a LR91 cell with the non-zero stimulus. This is a volume stimulus, ie
        * the function on the right-hand side of the first of the two bidomain equations.
        * An equal and opposite extra-cellular stimulus is implicitly enforced by the code,
        * which corresponds to having zero on the right-hand side of the second of the
        * bidomain equations.
        */

        return new GrayPathmanathan2016Rabbit(p_solver, mpStimulus); 
      
      } else {

        return new GrayPathmanathan2016Rabbit(p_solver, mpZeroStimulus);
      }
    }

/* We have no need for a destructor, since the problem class deals with deleting the cells. */
};

/*
* == Running the bidomain simulation ==
*
* Now we can define the test class, which must inherit from {{{CxxTest::TestSuite}}}
* as described in the writing basic tests tutorial. */
class TestChasteS1S2Bidomain : public CxxTest::TestSuite {

  public:
    // ** Methods **

    /* Methods wieth CxxTest need to start with Test so cmake can find them*/
    void TestSimpleSimulation() {
      std::cout<< "Here1  " << std::endl;

      /* Define the mesh, unit of length:  cm */
      double xLength=0.4;
      double yLength=0.0625;
      double zLength=0.0025;


      /* Define dx and dt, and other simulation parameters */
      std::cout<< "Here2  " << std::endl;
      double numSpacialPoints = 161-1;
      double h=xLength/numSpacialPoints;
      double dtCell = 0.001;
      double dtTissue =0.1;
      double timeDuration=7.0;

      // Number of output time steps, and size of output time step. Note that dtOut/dtTissue and dtOut/dtCell must be integers
      std::cout<< "Here3  " << std::endl;
      std::cout<< "h = " << h << std::endl;
      double numPrintTimeSteps=8-1;
      double dtOut = timeDuration/numPrintTimeSteps;


      /* The {{{HeartConfig}}} class is used to set various parameters (see the main ChasteGuides page
      * for information on default parameter values. Parameters in this file can be re-set
      * with {{{HeartConfig}}} if the user wishes, and other parameters such as end time must be set
      * using {{{HeartConfig}}}. Let us begin by setting the end time (in ms), the mesh to use, and the
      * output directory and filename-prefix. 
      */
      std::cout<< "Here4  " << std::endl;
      HeartConfig::Instance()->SetSimulationDuration(timeDuration); //ms
      TetrahedralMesh<3,3> mesh;
      mesh.ConstructRegularSlabMesh(h,xLength,yLength,zLength);
      HeartConfig::Instance()->SetOutputDirectory("Bidomain_3D_Feb15_-214500_161");
      HeartConfig::Instance()->SetOutputFilenamePrefix("results");
      HeartConfig::Instance()->SetVisualizerOutputPrecision(20);



      /*
      * It is possible to over-ride the default visualisation output (which is done during simulation
      * post-processing).
      */
      HeartConfig::Instance()->SetVisualizeWithMeshalyzer(true);


      /* To set the conductivity values
      *  in the principal fibre, sheet and normal directions do the following.
      * Note that {{{Create_c_vector}}} is just a helper method for creating a {{{c_vector<double,DIM>}}}
      * of the correct size (3, in this case). Make sure these methods are called before
      * {{{Initialise()}}}.
      */
      std::cout<< "Here5  " << std::endl;
      HeartConfig::Instance()->SetIntracellularConductivities(Create_c_vector(2.525, 0.222, 0.222));   // ms/cm
      HeartConfig::Instance()->SetExtracellularConductivities(Create_c_vector(8.21, 2.15, 2.15));   // ms/cm

      /* This is how to reset the surface-area-to-volume ratio and the capacitance.
      * (Here, we are actually just resetting them to their default values). */
      HeartConfig::Instance()->SetSurfaceAreaToVolumeRatio(1500); // 1/cm
      HeartConfig::Instance()->SetCapacitance(1); // uF/cm^2

      /* This is how to set the ode timestep (the timestep used to solve the cell models)
      * the pde timestep (the timestep used in solving the bidomain PDE), and the
      * printing timestep (how often the output is written to file). The defaults are
      * all 0.01, here we increase the printing timestep.
      */
      HeartConfig::Instance()->SetOdePdeAndPrintingTimeSteps(dtCell, dtTissue, dtOut);

      std::cout<< "Here6  " << std::endl;

      // Use Godunov OS method
      //HeartConfig::Instance()->SetUseReactionDiffusionOperatorSplittingBidomainSolver();
      HeartConfig::Instance()->SetUseReactionDiffusionOperatorSplitting();
      //HeartConfig::Instance()->SetUseGodunovReactionDiffusionOperatorSplittingBidomainSolver();


      /* Next, we have to create a cell factory of the type we defined above. */
      PointStimulus3dCellFactory cell_factory;

      /* Now we create a problem class using (a pointer to) the cell factory. */
      BidomainProblem<3> bidomain_problem( &cell_factory );

      std::cout<< "Here7  " << std::endl;


      /* Now we call Solve() to run the simulation. The output will be written to
      * `/tmp//testoutput/` in HDF5 format.  The
      * output will also be converted to selected visualiser formats at the end of the simulation.
      * Note that if you want to view the progress of longer simulations
      * go to the the output directory and look at the file
      * {{{progress_status.txt}}}, which will say the percentage of the
      * simulation run. */

      std::cout<< "Here8  " << std::endl;				

      bidomain_problem.SetMesh(&mesh);

      std::cout<< "Here9  " << std::endl;


      bidomain_problem.Initialise();


      std::cout<< "Here10  " << std::endl;

      bidomain_problem.Solve();


      std::cout<< "Here 11 " << std::endl;

    }
};

#endif /*TESTCHASTES1S2BIDOMAIN_HPP_*/




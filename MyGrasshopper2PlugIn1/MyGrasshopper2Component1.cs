using Grasshopper2.Components;
using Grasshopper2.UI;
using GrasshopperIO;
using Rhino.Geometry;
using System;

namespace MyGrasshopper2PlugIn1
{
    [IoId("b03bc156-c35d-4812-8685-570a9a40769c")]
    public sealed class MyGrasshopper2Component1 : Component
    {
        public MyGrasshopper2Component1() : base(new Nomen(
            "MyGrasshopper2Component1",
            "Description",
            "Chapter",
            "Section"))
        {

        }

        public MyGrasshopper2Component1(IReader reader) : base(reader) { }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void AddInputs(InputAdder inputs)
        {

        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void AddOutputs(OutputAdder outputs)
        {

        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="access">The IDataAccess object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void Process(IDataAccess access)
        {

        }
    }
}

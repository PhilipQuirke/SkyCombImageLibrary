using System.Windows.Forms;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SkyCombImageLibrary.ProcessLogic
{
    public class ProcessObjectParameters(bool triangulate, TextBox? output = null)
    {
        // This class is used to pass parameters to the process object
        public TextBox? Output = output;
        public bool Triangulate = triangulate;
    }
}

// Copyright SkyComb Limited 2024. All rights reserved.
using SkyCombDrone.DrawSpace;
using SkyCombImage.ProcessLogic;
using System.Drawing;


namespace SkyCombImage.DrawSpace
{
    // Code to draw stuff on images
    public class DrawFactory
    {
        public static DroneDrawGraph? Create(string useCase, ProcessAll process, ProcessDrawScope drawScope, Size size)
        {
            DroneDrawGraph? answer = null;

            switch (useCase)
            {
                case "altitude": answer = new ProcessDrawElevations(process, drawScope); break;
                case "speed": answer = new DrawSpeed(drawScope); break;
                case "pitch": answer = new DrawPitch(drawScope); break;
                case "deltayaw": answer = new DrawDeltaYaw(drawScope); break;
                case "roll": answer = new DrawRoll(drawScope); break;
                case "leg": answer = new DrawLeg(drawScope); break;
            }

            if (answer != null)
                answer.Initialise(size);

            return answer;
        }
    }
}

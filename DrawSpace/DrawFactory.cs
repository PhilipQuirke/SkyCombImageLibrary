// Copyright SkyComb Limited 2023. All rights reserved.
using System.Drawing;
using SkyCombImage.ProcessLogic;
using SkyCombDrone.DrawSpace;


namespace SkyCombImage.DrawSpace
{
    // Code to draw stuff on images
    public class DrawFactory
    {
        public static DroneDrawGraph Create(string useCase, ProcessDrawScope drawScope, Size size)
        {
            DroneDrawGraph answer = null;

            switch (useCase)
            {
                case "altitudebytime": answer = new CombDrawAltitudeByTime(null, drawScope); break;
                case "altitudebylinealm": answer = new CombDrawAltitudeByLinealM(null, drawScope); break;
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

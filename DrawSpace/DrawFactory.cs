// Copyright SkyComb Limited 2023. All rights reserved.
using System.Drawing;
using SkyCombImage.ProcessLogic;
using SkyCombDrone.DrawSpace;


namespace SkyCombImage.DrawSpace
{
    // Code to draw stuff on images
    public class DrawFactory
    {
        public static DrawGraph Create(string useCase, DrawScope drawScope, Size size)
        {
            DrawGraph answer = null;

            switch (useCase)
            {
                case "altitudebytime": answer = new DrawCombAltitudeByTime(null, drawScope); break;
                case "altitudebylinealm": answer = new DrawCombAltitudeByLinealM(null, drawScope); break;
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

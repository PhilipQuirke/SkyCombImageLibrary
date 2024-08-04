// Copyright SkyComb Limited 2024. All rights reserved. 
using SkyCombGround.CommonSpace;


namespace SkyCombImage.ProcessModel
{
    // The Video, Drone & model scope of a processing run
    public class ProcessScopeModel : ConfigBase
    {
        // INPUT VIDEO 
        // First input video frame to process 
        public int FirstInputFrameId { get; set; } = UnknownValue;
        // Last input video frame to process 
        public int LastInputFrameId { get; set; } = UnknownValue;
        // First millisecond of input video to process 
        public int FirstVideoFrameMs { get; set; } = UnknownValue;
        // Last millisecond of input video to process 
        public int LastVideoFrameMs { get; set; } = UnknownValue;
        // Current input video frame Id. No offsets applied - straight from input video.
        public int CurrInputFrameId { get; set; } = UnknownValue;
        // Current input video position in milliseconds. No offsets applied - straight from input video.
        public int CurrInputFrameMs { get; set; } = UnknownValue;
        // Duration of video to process in milliseconds
        public int InputVideoDurationMs { get { return LastVideoFrameMs - FirstVideoFrameMs; } }


        // DISPLAY VIDEO (if any)
        // Current display video frame Id. No offsets applied - straight from display video.
        public int CurrDisplayFrameId { get; set; } = UnknownValue;
        // Current display video position in milliseconds. No offsets applied - straight from display video.
        public int CurrDisplayFrameMs { get; set; } = UnknownValue;


        // DRONE
        // StepId of current flight step
        public int CurrRunStepId { get; set; } = UnknownValue;
        // LegId of current flight step (if any)
        public int CurrRunLegId { get; set; } = UnknownValue;
        public string CurrRunLegName { get { return IdToLetter(CurrRunLegId); } }


        // MODEL
        // First model block. One-based.
        public const int FirstBlockId = 1;
        // Last model block. One-based.
        public int LastBlockId { get { return LastInputFrameId - FirstInputFrameId + 1; } }
        // Current model block. One-based.
        public int CurrBlockId { get; set; } = 1;


        public ProcessScopeModel()
        {
        }
        public ProcessScopeModel(ProcessScopeModel other)
        {
            CopyScope(other);
        }


        public void CopyScope(ProcessScopeModel other)
        {
            FirstInputFrameId = other.FirstInputFrameId;
            LastInputFrameId = other.LastInputFrameId;
            FirstVideoFrameMs = other.FirstVideoFrameMs;
            LastVideoFrameMs = other.LastVideoFrameMs;
            CurrInputFrameId = other.CurrInputFrameId;
            CurrInputFrameMs = other.CurrInputFrameMs;
            CurrDisplayFrameId = other.CurrDisplayFrameId;
            CurrDisplayFrameMs = other.CurrDisplayFrameMs;

            CurrRunStepId = other.CurrRunStepId;
            CurrRunLegId = other.CurrRunLegId;

            CurrBlockId = other.CurrBlockId;
        }
    }
}

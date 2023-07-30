// Copyright SkyComb Limited 2023. All rights reserved. 
using SkyCombGround.CommonSpace;


namespace SkyCombImage.CategorySpace
{
    public class CategoryFactory : BaseConstants
    {
        public static MasterCategoryModel NewMasterCategoryModel(List<string> settings)
        {
            return new MasterCategoryModel(settings);
        }


        public static ObjectCategoryModel NewObjectCategoryModel(List<string> settings)
        {
            return new ObjectCategoryModel(settings);
        }
    }
}

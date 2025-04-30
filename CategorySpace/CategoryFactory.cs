// Copyright SkyComb Limited 2024. All rights reserved. 


namespace SkyCombImage.CategorySpace
{
    public class ObjectCategoryFactory
    {
        public static ObjectCategoryModel NewObjectCategoryModel(List<string> settings)
        {
            return new ObjectCategoryModel(settings);
        }
    }
}

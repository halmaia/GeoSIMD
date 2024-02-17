namespace GeoSIMD
{
    internal class Program
    {
        static void Main(string[] args)
        {
            double[] a = [double.NaN, 2, 3, 4, 1, 2, 3, 4, 1, 2];
           var min= GeoSIMD.Min(a);
        }
    }
}

namespace GeoSIMD
{
    internal class Program
    {

        static void Main(string[] args)
        {
            double[] a = [double.NaN, 40, 0, 99, 1, 2, 3, 4, 7, 2];
            double[] b = new double[a.Length];
            a.CopyTo(b, 0);
           GeoSIMD.WGS84_WebMercator(a,b);

        }
    }
}

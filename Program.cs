namespace GeoSIMD
{
    internal class Program
    {
        static void Main(string[] args)
        {
            double[] a = [1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4];
            Span<double> x = [0, 1, 1, 0], y = [0, 0, 1, 1];
            GeoSIMD.ClosePolygon(ref x, ref y);
        }
    }
}

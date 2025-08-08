// this is a C# console application
// create a console app in visual studio and replace the starter code with this
// except for the namespace
// you also need to put in an access token to download tiles from Azure
// this requires setting up an Azure student account and an endpoint
// (don't remember the exact process, it's been a while since I did it, ask ChatGPT)

namespace TileDownloader
{
    internal class Program
    {
        private const string AccessToken = "";

        public static async Task DownloadTile(int x, int y, int zoom, string outputPath) {
            string url = $"https://atlas.microsoft.com/map/imagery/png?api-version=1.0&style=satellite&zoom={zoom}&x={x}&y={y}&subscription-key={AccessToken}";

            using HttpClient client = new HttpClient();

            try {
                byte[] imageBytes = await client.GetByteArrayAsync(url);
                await File.WriteAllBytesAsync(outputPath, imageBytes);
                //Console.WriteLine($"Tile {x},{y} saved to {outputPath}");
            } catch (Exception ex) {
                Console.WriteLine($"Failed to download tile {x},{y}: {ex.Message}");
            }
        }

        private static int LonToTileX(double lon, int zoom) {
            return (int)((lon + 180.0) / 360.0 * (1 << zoom));
        }

        private static int LatToTileY(double lat, int zoom) {
            double latRad = lat * Math.PI / 180.0;
            return (int)((1.0 - Math.Log(Math.Tan(latRad) + 1.0 / Math.Cos(latRad)) / Math.PI) / 2.0 * (1 << zoom));
        }

        static async Task Main(string[] args) {
            double lat = 51.42254;
            double lon = -112.64081;
            int zoom = 19;

            int centerX = LonToTileX(lon, zoom);
            int centerY = LatToTileY(lat, zoom);

            int mapWidth = 110;
            int half = mapWidth / 2;
            int counter = 0;

            List<Task> tasks = new List<Task>();

            for (int dx = -half; dx < half; dx++) {
                for (int dy = -half; dy < half; dy++) {
                    counter++;

                    int tileX = centerX + dx;
                    int tileY = centerY + dy;
                    string outputPath = $@"C:\Users\matt\Documents\GitHub\Rover-Unity\Assets\Control Interface App\Maps\Drumheller\~{dx},{dy}~.png";
                    if (File.Exists(outputPath)) continue;
                    Console.WriteLine($"{counter}/{mapWidth * mapWidth}");

                    tasks.Add(DownloadTile(tileX, tileY, zoom, outputPath));

                    if (tasks.Count == 9) {
                        await Task.WhenAll(tasks);
                        tasks.Clear();
                    }

                    if (counter % 240 == 0) {
                        Console.WriteLine("Waiting...");
                        await Task.Delay(1000);
                    }
                }
            }

            // Await any remaining tasks
            if (tasks.Count > 0) {
                await Task.WhenAll(tasks);
            }
        }
    }
}

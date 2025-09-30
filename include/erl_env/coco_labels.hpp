#pragma once

#include "erl_common/json.hpp"

#include <map>
#include <unordered_map>

namespace erl::env {
    struct CocoObjectLabels2017 {
        // copied from instances_val2017.json in COCO 2017 dataset:
        // https://cocodataset.org/#download format: https://cocodataset.org/#format-data
        inline static const char* json =
            "{\"categories\": [{\"supercategory\": \"person\",\"id\": 1,\"name\": "
            "\"person\"},{\"supercategory\": \"vehicle\",\"id\": 2,\"name\": "
            "\"bicycle\"},{\"supercategory\": \"vehicle\",\"id\": 3,\"name\": "
            "\"car\"},{\"supercategory\": \"vehicle\",\"id\": 4,\"name\": "
            "\"motorcycle\"},{\"supercategory\": \"vehicle\",\"id\": 5,\"name\": "
            "\"airplane\"},{\"supercategory\": \"vehicle\",\"id\": 6,\"name\": "
            "\"bus\"},{\"supercategory\": \"vehicle\",\"id\": 7,\"name\": "
            "\"train\"},{\"supercategory\": \"vehicle\",\"id\": 8,\"name\": "
            "\"truck\"},{\"supercategory\": \"vehicle\",\"id\": 9,\"name\": "
            "\"boat\"},{\"supercategory\": \"outdoor\",\"id\": 10,\"name\": \"traffic "
            "light\"},{\"supercategory\": \"outdoor\",\"id\": 11,\"name\": \"fire "
            "hydrant\"},{\"supercategory\": \"outdoor\",\"id\": 13,\"name\": \"stop "
            "sign\"},{\"supercategory\": \"outdoor\",\"id\": 14,\"name\": \"parking "
            "meter\"},{\"supercategory\": \"outdoor\",\"id\": 15,\"name\": "
            "\"bench\"},{\"supercategory\": \"animal\",\"id\": 16,\"name\": "
            "\"bird\"},{\"supercategory\": \"animal\",\"id\": 17,\"name\": "
            "\"cat\"},{\"supercategory\": \"animal\",\"id\": 18,\"name\": "
            "\"dog\"},{\"supercategory\": \"animal\",\"id\": 19,\"name\": "
            "\"horse\"},{\"supercategory\": \"animal\",\"id\": 20,\"name\": "
            "\"sheep\"},{\"supercategory\": \"animal\",\"id\": 21,\"name\": "
            "\"cow\"},{\"supercategory\": \"animal\",\"id\": 22,\"name\": "
            "\"elephant\"},{\"supercategory\": \"animal\",\"id\": 23,\"name\": "
            "\"bear\"},{\"supercategory\": \"animal\",\"id\": 24,\"name\": "
            "\"zebra\"},{\"supercategory\": \"animal\",\"id\": 25,\"name\": "
            "\"giraffe\"},{\"supercategory\": \"accessory\",\"id\": 27,\"name\": "
            "\"backpack\"},{\"supercategory\": \"accessory\",\"id\": 28,\"name\": "
            "\"umbrella\"},{\"supercategory\": \"accessory\",\"id\": 31,\"name\": "
            "\"handbag\"},{\"supercategory\": \"accessory\",\"id\": 32,\"name\": "
            "\"tie\"},{\"supercategory\": \"accessory\",\"id\": 33,\"name\": "
            "\"suitcase\"},{\"supercategory\": \"sports\",\"id\": 34,\"name\": "
            "\"frisbee\"},{\"supercategory\": \"sports\",\"id\": 35,\"name\": "
            "\"skis\"},{\"supercategory\": \"sports\",\"id\": 36,\"name\": "
            "\"snowboard\"},{\"supercategory\": \"sports\",\"id\": 37,\"name\": \"sports "
            "ball\"},{\"supercategory\": \"sports\",\"id\": 38,\"name\": "
            "\"kite\"},{\"supercategory\": \"sports\",\"id\": 39,\"name\": \"baseball "
            "bat\"},{\"supercategory\": \"sports\",\"id\": 40,\"name\": \"baseball "
            "glove\"},{\"supercategory\": \"sports\",\"id\": 41,\"name\": "
            "\"skateboard\"},{\"supercategory\": \"sports\",\"id\": 42,\"name\": "
            "\"surfboard\"},{\"supercategory\": \"sports\",\"id\": 43,\"name\": \"tennis "
            "racket\"},{\"supercategory\": \"kitchen\",\"id\": 44,\"name\": "
            "\"bottle\"},{\"supercategory\": \"kitchen\",\"id\": 46,\"name\": \"wine "
            "glass\"},{\"supercategory\": \"kitchen\",\"id\": 47,\"name\": "
            "\"cup\"},{\"supercategory\": \"kitchen\",\"id\": 48,\"name\": "
            "\"fork\"},{\"supercategory\": \"kitchen\",\"id\": 49,\"name\": "
            "\"knife\"},{\"supercategory\": \"kitchen\",\"id\": 50,\"name\": "
            "\"spoon\"},{\"supercategory\": \"kitchen\",\"id\": 51,\"name\": "
            "\"bowl\"},{\"supercategory\": \"food\",\"id\": 52,\"name\": "
            "\"banana\"},{\"supercategory\": \"food\",\"id\": 53,\"name\": "
            "\"apple\"},{\"supercategory\": \"food\",\"id\": 54,\"name\": "
            "\"sandwich\"},{\"supercategory\": \"food\",\"id\": 55,\"name\": "
            "\"orange\"},{\"supercategory\": \"food\",\"id\": 56,\"name\": "
            "\"broccoli\"},{\"supercategory\": \"food\",\"id\": 57,\"name\": "
            "\"carrot\"},{\"supercategory\": \"food\",\"id\": 58,\"name\": \"hot "
            "dog\"},{\"supercategory\": \"food\",\"id\": 59,\"name\": "
            "\"pizza\"},{\"supercategory\": \"food\",\"id\": 60,\"name\": "
            "\"donut\"},{\"supercategory\": \"food\",\"id\": 61,\"name\": "
            "\"cake\"},{\"supercategory\": \"furniture\",\"id\": 62,\"name\": "
            "\"chair\"},{\"supercategory\": \"furniture\",\"id\": 63,\"name\": "
            "\"couch\"},{\"supercategory\": \"furniture\",\"id\": 64,\"name\": \"potted "
            "plant\"},{\"supercategory\": \"furniture\",\"id\": 65,\"name\": "
            "\"bed\"},{\"supercategory\": \"furniture\",\"id\": 67,\"name\": \"dining "
            "table\"},{\"supercategory\": \"furniture\",\"id\": 70,\"name\": "
            "\"toilet\"},{\"supercategory\": \"electronic\",\"id\": 72,\"name\": "
            "\"tv\"},{\"supercategory\": \"electronic\",\"id\": 73,\"name\": "
            "\"laptop\"},{\"supercategory\": \"electronic\",\"id\": 74,\"name\": "
            "\"mouse\"},{\"supercategory\": \"electronic\",\"id\": 75,\"name\": "
            "\"remote\"},{\"supercategory\": \"electronic\",\"id\": 76,\"name\": "
            "\"keyboard\"},{\"supercategory\": \"electronic\",\"id\": 77,\"name\": \"cell "
            "phone\"},{\"supercategory\": \"appliance\",\"id\": 78,\"name\": "
            "\"microwave\"},{\"supercategory\": \"appliance\",\"id\": 79,\"name\": "
            "\"oven\"},{\"supercategory\": \"appliance\",\"id\": 80,\"name\": "
            "\"toaster\"},{\"supercategory\": \"appliance\",\"id\": 81,\"name\": "
            "\"sink\"},{\"supercategory\": \"appliance\",\"id\": 82,\"name\": "
            "\"refrigerator\"},{\"supercategory\": \"indoor\",\"id\": 84,\"name\": "
            "\"book\"},{\"supercategory\": \"indoor\",\"id\": 85,\"name\": "
            "\"clock\"},{\"supercategory\": \"indoor\",\"id\": 86,\"name\": "
            "\"vase\"},{\"supercategory\": \"indoor\",\"id\": 87,\"name\": "
            "\"scissors\"},{\"supercategory\": \"indoor\",\"id\": 88,\"name\": \"teddy "
            "bear\"},{\"supercategory\": \"indoor\",\"id\": 89,\"name\": \"hair "
            "drier\"},{\"supercategory\": \"indoor\",\"id\": 90,\"name\": \"toothbrush\"}]}";

        inline static std::map<int, std::string> obj_id_to_label = {};
        inline static std::unordered_map<std::string, int> obj_label_to_id = {};
        inline static std::map<int, std::string> obj_id_to_supercategory =
            []() -> std::map<int, std::string> {
            auto json = nlohmann::json::parse(CocoObjectLabels2017::json);
            std::map<int, std::string> obj_id_to_supercategory;
            for (auto& cat: json["categories"]) {
                CocoObjectLabels2017::obj_id_to_label[cat["id"].get<int>()] =
                    cat["name"].get<std::string>();
                CocoObjectLabels2017::obj_label_to_id[cat["name"].get<std::string>()] =
                    cat["id"].get<int>();
                obj_id_to_supercategory[cat["id"].get<int>()] =
                    cat["supercategory"].get<std::string>();
            }
            return obj_id_to_supercategory;
        }();
    };

    struct CocoStuffLabels {
        inline static const char* json =
            "{\"categories\": [{\"supercategory\": \"textile\", \"id\": 92, \"name\": \"banner\"}, "
            "{\"supercategory\": \"textile\", \"id\": 93, \"name\": "
            "\"blanket\"}, {\"supercategory\": \"plant\", \"id\": 94, \"name\": \"branch\"}, "
            "{\"supercategory\": \"building\", \"id\": 95, \"name\": "
            "\"bridge\"}, {\"supercategory\": \"building\", \"id\": 96, \"name\": "
            "\"building-other\"}, {\"supercategory\": \"plant\", \"id\": 97, \"name\": "
            "\"bush\"}, {\"supercategory\": \"furniture-stuff\", \"id\": 98, \"name\": "
            "\"cabinet\"}, {\"supercategory\": \"structural\", \"id\": 99, \"name\": "
            "\"cage\"}, {\"supercategory\": \"raw-material\", \"id\": 100, \"name\": "
            "\"cardboard\"}, {\"supercategory\": \"floor\", \"id\": 101, \"name\": "
            "\"carpet\"}, {\"supercategory\": \"ceiling\", \"id\": 102, \"name\": "
            "\"ceiling-other\"}, {\"supercategory\": \"ceiling\", \"id\": 103, \"name\": "
            "\"ceiling-tile\"}, {\"supercategory\": \"textile\", \"id\": 104, \"name\": "
            "\"cloth\"}, {\"supercategory\": \"textile\", \"id\": 105, \"name\": "
            "\"clothes\"}, {\"supercategory\": \"sky\", \"id\": 106, \"name\": \"clouds\"}, "
            "{\"supercategory\": \"furniture-stuff\", \"id\": 107, \"name\": "
            "\"counter\"}, {\"supercategory\": \"furniture-stuff\", \"id\": 108, \"name\": "
            "\"cupboard\"}, {\"supercategory\": \"textile\", \"id\": 109, "
            "\"name\": \"curtain\"}, {\"supercategory\": \"furniture-stuff\", \"id\": 110, "
            "\"name\": \"desk-stuff\"}, {\"supercategory\": \"ground\", \"id\": "
            "111, \"name\": \"dirt\"}, {\"supercategory\": \"furniture-stuff\", \"id\": 112, "
            "\"name\": \"door-stuff\"}, {\"supercategory\": \"structural\", "
            "\"id\": 113, \"name\": \"fence\"}, {\"supercategory\": \"floor\", \"id\": 114, "
            "\"name\": \"floor-marble\"}, {\"supercategory\": \"floor\", "
            "\"id\": 115, \"name\": \"floor-other\"}, {\"supercategory\": \"floor\", \"id\": 116, "
            "\"name\": \"floor-stone\"}, {\"supercategory\": \"floor\", "
            "\"id\": 117, \"name\": \"floor-tile\"}, {\"supercategory\": \"floor\", \"id\": 118, "
            "\"name\": \"floor-wood\"}, {\"supercategory\": \"plant\", "
            "\"id\": 119, \"name\": \"flower\"}, {\"supercategory\": \"water\", \"id\": 120, "
            "\"name\": \"fog\"}, {\"supercategory\": \"food-stuff\", \"id\": "
            "121, \"name\": \"food-other\"}, {\"supercategory\": \"food-stuff\", \"id\": 122, "
            "\"name\": \"fruit\"}, {\"supercategory\": \"furniture-stuff\", "
            "\"id\": 123, \"name\": \"furniture-other\"}, {\"supercategory\": \"plant\", \"id\": "
            "124, \"name\": \"grass\"}, {\"supercategory\": \"ground\", "
            "\"id\": 125, \"name\": \"gravel\"}, {\"supercategory\": \"ground\", \"id\": 126, "
            "\"name\": \"ground-other\"}, {\"supercategory\": \"solid\", "
            "\"id\": 127, \"name\": \"hill\"}, {\"supercategory\": \"building\", \"id\": 128, "
            "\"name\": \"house\"}, {\"supercategory\": \"plant\", \"id\": "
            "129, \"name\": \"leaves\"}, {\"supercategory\": \"furniture-stuff\", \"id\": 130, "
            "\"name\": \"light\"}, {\"supercategory\": \"textile\", \"id\": "
            "131, \"name\": \"mat\"}, {\"supercategory\": \"raw-material\", \"id\": 132, \"name\": "
            "\"metal\"}, {\"supercategory\": \"furniture-stuff\", "
            "\"id\": 133, \"name\": \"mirror-stuff\"}, {\"supercategory\": \"plant\", \"id\": 134, "
            "\"name\": \"moss\"}, {\"supercategory\": \"solid\", \"id\": "
            "135, \"name\": \"mountain\"}, {\"supercategory\": \"ground\", \"id\": 136, \"name\": "
            "\"mud\"}, {\"supercategory\": \"textile\", \"id\": 137, "
            "\"name\": \"napkin\"}, {\"supercategory\": \"structural\", \"id\": 138, \"name\": "
            "\"net\"}, {\"supercategory\": \"raw-material\", \"id\": 139, "
            "\"name\": \"paper\"}, {\"supercategory\": \"ground\", \"id\": 140, \"name\": "
            "\"pavement\"}, {\"supercategory\": \"textile\", \"id\": 141, "
            "\"name\": \"pillow\"}, {\"supercategory\": \"plant\", \"id\": 142, \"name\": "
            "\"plant-other\"}, {\"supercategory\": \"raw-material\", \"id\": 143, "
            "\"name\": \"plastic\"}, {\"supercategory\": \"ground\", \"id\": 144, \"name\": "
            "\"platform\"}, {\"supercategory\": \"ground\", \"id\": 145, "
            "\"name\": \"playingfield\"}, {\"supercategory\": \"structural\", \"id\": 146, "
            "\"name\": \"railing\"}, {\"supercategory\": \"ground\", \"id\": "
            "147, \"name\": \"railroad\"}, {\"supercategory\": \"water\", \"id\": 148, \"name\": "
            "\"river\"}, {\"supercategory\": \"ground\", \"id\": 149, "
            "\"name\": \"road\"}, {\"supercategory\": \"solid\", \"id\": 150, \"name\": \"rock\"}, "
            "{\"supercategory\": \"building\", \"id\": 151, \"name\": "
            "\"roof\"}, {\"supercategory\": \"textile\", \"id\": 152, \"name\": \"rug\"}, "
            "{\"supercategory\": \"food-stuff\", \"id\": 153, \"name\": "
            "\"salad\"}, {\"supercategory\": \"ground\", \"id\": 154, \"name\": \"sand\"}, "
            "{\"supercategory\": \"water\", \"id\": 155, \"name\": \"sea\"}, "
            "{\"supercategory\": \"furniture-stuff\", \"id\": 156, \"name\": \"shelf\"}, "
            "{\"supercategory\": \"sky\", \"id\": 157, \"name\": \"sky-other\"}, "
            "{\"supercategory\": \"building\", \"id\": 158, \"name\": \"skyscraper\"}, "
            "{\"supercategory\": \"ground\", \"id\": 159, \"name\": \"snow\"}, "
            "{\"supercategory\": \"solid\", \"id\": 160, \"name\": \"solid-other\"}, "
            "{\"supercategory\": \"furniture-stuff\", \"id\": 161, \"name\": "
            "\"stairs\"}, {\"supercategory\": \"solid\", \"id\": 162, \"name\": \"stone\"}, "
            "{\"supercategory\": \"plant\", \"id\": 163, \"name\": \"straw\"}, "
            "{\"supercategory\": \"structural\", \"id\": 164, \"name\": \"structural-other\"}, "
            "{\"supercategory\": \"furniture-stuff\", \"id\": 165, \"name\": "
            "\"table\"}, {\"supercategory\": \"building\", \"id\": 166, \"name\": \"tent\"}, "
            "{\"supercategory\": \"textile\", \"id\": 167, \"name\": "
            "\"textile-other\"}, {\"supercategory\": \"textile\", \"id\": 168, \"name\": "
            "\"towel\"}, {\"supercategory\": \"plant\", \"id\": 169, \"name\": "
            "\"tree\"}, {\"supercategory\": \"food-stuff\", \"id\": 170, \"name\": \"vegetable\"}, "
            "{\"supercategory\": \"wall\", \"id\": 171, \"name\": "
            "\"wall-brick\"}, {\"supercategory\": \"wall\", \"id\": 172, \"name\": "
            "\"wall-concrete\"}, {\"supercategory\": \"wall\", \"id\": 173, \"name\": "
            "\"wall-other\"}, {\"supercategory\": \"wall\", \"id\": 174, \"name\": "
            "\"wall-panel\"}, {\"supercategory\": \"wall\", \"id\": 175, \"name\": "
            "\"wall-stone\"}, {\"supercategory\": \"wall\", \"id\": 176, \"name\": \"wall-tile\"}, "
            "{\"supercategory\": \"wall\", \"id\": 177, \"name\": "
            "\"wall-wood\"}, {\"supercategory\": \"water\", \"id\": 178, \"name\": "
            "\"water-other\"}, {\"supercategory\": \"water\", \"id\": 179, \"name\": "
            "\"waterdrops\"}, {\"supercategory\": \"window\", \"id\": 180, \"name\": "
            "\"window-blind\"}, {\"supercategory\": \"window\", \"id\": 181, \"name\": "
            "\"window-other\"}, {\"supercategory\": \"solid\", \"id\": 182, \"name\": \"wood\"}, "
            "{\"supercategory\": \"other\", \"id\": 183, \"name\": "
            "\"other\"}]}";
        // copied from https://github.com/nightrome/cocostuff/blob/master/labels.md?plain=1
        inline static std::map<int, std::string> stuff_id_to_label =
            []() -> std::map<int, std::string> {
            return {
                {0, "unlabeled"},  // Pixels that do not belong to any of the other classes
                {1, "person"},
                {2, "bicycle"},
                {3, "car"},
                {4, "motorcycle"},
                {5, "airplane"},
                {6, "bus"},
                {7, "train"},
                {8, "truck"},
                {9, "boat"},
                {10, "traffic light"},
                {11, "fire hydrant"},
                {12, "street sign"},  // Removed from COCO.
                {13, "stop sign"},
                {14, "parking meter"},
                {15, "bench"},
                {16, "bird"},
                {17, "cat"},
                {18, "dog"},
                {19, "horse"},
                {20, "sheep"},
                {21, "cow"},
                {22, "elephant"},
                {23, "bear"},
                {24, "zebra"},
                {25, "giraffe"},
                {26, "hat"},  // Removed from COCO.
                {27, "backpack"},
                {28, "umbrella"},
                {29, "shoe"},         // Removed from COCO.
                {30, "eye glasses"},  // Removed from COCO.
                {31, "handbag"},
                {32, "tie"},
                {33, "suitcase"},
                {34, "frisbee"},
                {35, "skis"},
                {36, "snowboard"},
                {37, "sports ball"},
                {38, "kite"},
                {39, "baseball bat"},
                {40, "baseball glove"},
                {41, "skateboard"},
                {42, "surfboard"},
                {43, "tennis racket"},
                {44, "bottle"},
                {45, "plate"},  // Removed from COCO.
                {46, "wine glass"},
                {47, "cup"},
                {48, "fork"},
                {49, "knife"},
                {50, "spoon"},
                {51, "bowl"},
                {52, "banana"},
                {53, "apple"},
                {54, "sandwich"},
                {55, "orange"},
                {56, "broccoli"},
                {57, "carrot"},
                {58, "hot dog"},
                {59, "pizza"},
                {60, "donut"},
                {61, "cake"},
                {62, "chair"},
                {63, "couch"},
                {64, "potted plant"},
                {65, "bed"},
                {66, "mirror"},  // Removed from COCO.
                {67, "dining table"},
                {68, "window"},  // Removed from COCO.
                {69, "desk"},    // Removed from COCO.
                {70, "toilet"},
                {71, "door"},  // Removed from COCO.
                {72, "tv"},
                {73, "laptop"},
                {74, "mouse"},
                {75, "remote"},
                {76, "keyboard"},
                {77, "cell phone"},
                {78, "microwave"},
                {79, "oven"},
                {80, "toaster"},
                {81, "sink"},
                {82, "refrigerator"},
                {83, "blender"},  // Removed from COCO.
                {84, "book"},
                {85, "clock"},
                {86, "vase"},
                {87, "scissors"},
                {88, "teddy bear"},
                {89, "hair drier"},
                {90, "toothbrush"},
                {91, "hair brush"},  // Removed from COCO.
                {92, "banner"},   // Any large sign, especially if constructed of soft material or
                                  // fabric, often seen in stadiums and advertising.
                {93, "blanket"},  // A loosely woven fabric, used for warmth while sleeping.
                {94, "branch"},   // The woody part of a tree or bush, arising from the trunk and
                                  // usually dividing.
                {95, "bridge"},   // A manmade construction that spans a divide (incl. train bridge,
                                  // river bridge).
                {96, "building-other"},  // Any other type of building or structures.
                {97, "bush"},  // A woody plant distinguished from a tree by its multiple stems and
                               // lower height (incl. hedge, scrub).
                {98, "cabinet"},     // A storage closet, often hanging on the wall.
                {99, "cage"},        // An enclosure made of bars, often seen in zoos.
                {100, "cardboard"},  // A wood-based material resembling heavy paper, used in the
                                     // manufacture of boxes, cartons and signs.
                {101, "carpet"},     // A fabric used as a floor covering.
                {102, "ceiling-other"},  // Other types of ceilings (incl. industrial ceilings,
                                         // painted ceilings).
                {103, "ceiling-tile"},   // A ceiling made of regularly-shaped slabs.
                {104, "cloth"},  // A piece of cloth used for a particular purpose. (incl. cleaning
                                 // cloth).
                {105, "clothes"},   // Items of clothing or apparel, not currently worn by a person.
                {106, "clouds"},    // A visible mass of water droplets suspended in the air.
                {107, "counter"},   // A surface in the kitchen or bathroom, often built into a wall
                                    // or above a cabinet, which holds the wash-basin or surface to
                                    // prepare food.
                {108, "cupboard"},  // A piece of furniture used for storing dishware or a wardrobe
                                    // for clothes, sometimes hanging on the wall.
                {109, "curtain"},   // A piece of cloth covering a window, bed or shower to offer
                                    // privacy and keep out light.
                {110, "desk-stuff"},  // A piece of furniture with a flat surface and typically with
                                      // drawers, at which one can read, write, or do other work.
                {111, "dirt"},        // Soil or earth (incl. dirt path).
                {112, "door-stuff"},  // A portal of entry into a building, room or vehicle,
                                      // consisting of a rigid plane movable on a hinge (incl. the
                                      // frame, replaces door).
                {113,
                 "fence"},  // A thin, human-constructed barrier which separates two pieces of land.
                {114,
                 "floor-marble"},  // The supporting surface of a room or outside, made of marble.
                {115, "floor-other"},  // Any other type of floor (incl. rubber-based floor).
                {116, "floor-stone"},  // The supporting surface of a room or outside, made of stone
                                       // (incl. brick floor).
                {117, "floor-tile"},   // The supporting surface of a room or outside, made of
                                       // regularly-shaped slabs (incl. tiled stone floor, tiled
                                       // marble floor).
                {118, "floor-wood"},   // The supporting surface of a room or outside, made of wood
                                       // (incl. wooden tiles, parquet, laminate, wooden boards).
                {119, "flower"},  // The seed-bearing part of a plant (incl. the entire flower).
                {120, "fog"},  // A thick cloud of tiny water droplets suspended in the atmosphere
                               // near the earth's surface.
                {121, "food-other"},       // Any other type of food.
                {122, "fruit"},            // The sweet and fleshy product of a tree or other plant.
                {123, "furniture-other"},  // Any other type of furniture (incl. oven).
                {124, "grass"},   // Vegetation consisting of typically short plants with long,
                                  // narrow leaves (incl. lawn, pasture).
                {125, "gravel"},  // A loose aggregation of small water-worn or pounded stones.
                {126, "ground-other"},  // Any other type of ground found outside a building.
                {127, "hill"},   // A naturally raised area of land, not as high as a mountain,
                                 // viewed at a distance and may be covered in trees, snow or grass.
                {128, "house"},  // A smaller size building for human habitation.
                {129, "leaves"},  // A structure of a higher plant, typically green and blade-like,
                                  // that is attached to a stem or stalk.
                {130,
                 "light"},  // A source of illumination, especially a lamp (incl. ceiling lights).
                {131, "mat"},    // A piece of coarse material placed on a floor for people to wipe
                                 // their feet on.
                {132, "metal"},  // A raw metal material (incl. a pile of metal).
                {133, "mirror-stuff"},  // A glass coated surface which reflects a clear image
                                        // (incl. the frame, replaces mirror).
                {134, "moss"},  // A small flowerless green plant which lacks true roots, growing in
                                // in damp habitats.
                {135, "mountain"},  // A large natural elevation rising abruptly from the
                                    // surrounding level, viewed at a distance and may be covered in
                                    // trees, snow or grass.
                {136,
                 "mud"},  // A soft, sticky matter resulting from the mixing of earth and water.
                {137, "napkin"},  // A piece of cloth or paper used at a meal to wipe the fingers or
                                  // lips.
                {138, "net"},     // An open-meshed fabric twisted, knotted, or woven together at
                                  // regular intervals.
                {139, "paper"},   // A material manufactured in thin sheets from the pulp of wood.
                {140, "pavement"},  // A typically raised paved path for pedestrians at the side of
                                    // a road.
                {141, "pillow"},  // A rectangular cloth bag stuffed with soft materials to support
                                  // the head.
                {142, "plant-other"},  // Any other type of plant.
                {143, "plastic"},      // Raw plastic material.
                {144, "platform"},     // A raised level surface on which people or things can stand
                                       // (incl. railroad platform).
                {145, "playingfield"},  // A ground marked off for various games (incl. indoor and
                                        // outdoor).
                {146, "railing"},       // A fence or barrier made of typically metal rails.
                {147, "railroad"},  // A track made of steel rails along which trains run (incl. the
                                    // wooden beams).
                {148, "river"},     // A stream of flowing water.
                {149, "road"},      // A paved way leading from one place to another.
                {150,
                 "rock"},  // The solid mineral material forming part of the surface of the earth.
                {151, "roof"},  // The structure forming the upper covering of a building.
                {152, "rug"},   // A floor covering of thick woven material, typically not extending
                                // over the entire floor.
                {153, "salad"},      // A cold dish of various mixtures of raw or cooked vegetables.
                {154, "sand"},       // A loose granular substance, typically pale yellowish brown,
                                     // resulting from erosion (incl. beach).
                {155, "sea"},        // Expanse of water that covers most of the earth's surface.
                {156, "shelf"},      // An open piece of furniture that provides a surface for the
                                     // storage or display of objects.
                {157, "sky-other"},  // Any other type of sky (incl. blue sky).
                {158, "skyscraper"},  // A very tall building of many storeys.
                {159, "snow"},  // Atmospheric water vapour frozen into ice crystals, falling or
                                // lying on the ground.
                {160, "solid-other"},  // Any other type of solid material.
                {161, "stairs"},  // A set of steps leading from one floor to another (incl. stairs
                                  // inside or outside a building).
                {162, "stone"},   // A piece of stone shaped for a purpose.
                {163, "straw"},   // Dried stalks of grain.
                {164, "structural-other"},  // Any other type of structural connection (incl. arcs,
                                            // pillars).
                {165, "table"},  // A piece of furniture with a flat top and one or more legs.
                {166, "tent"},   // A portable shelter made of cloth.
                {167, "textile-other"},  // Any other type of textile.
                {168, "towel"},  // A piece of thick absorbent cloth used for drying oneself.
                {169, "tree"},   // A woody plant, typically having a single trunk growing to a
                                // considerable height and bearing lateral branches at some distance
                                // from the ground
                {170, "vegetable"},      // A part of a plant used as food.
                {171, "wall-brick"},     // A building wall made of bricks of clay.
                {172, "wall-concrete"},  // A building wall made of concrete.
                {173, "wall-other"},     // Any other type of wall.
                {174, "wall-panel"},     // A panel that is attached to a wall.
                {175, "wall-stone"},     // A building wall made of stone.
                {176, "wall-tile"},  // A building wall made of tiles, such as used in bathrooms and
                                     // kitchens.
                {177, "wall-wood"},  // A building wall made of wooden material.
                {178, "water-other"},  // Any other type of water (incl. lake).
                {179, "waterdrops"},   // Sprinkles or drops of water not connected to a larger body
                                       // of water.
                {180, "window-blind"},  // Blinds and shutters that cover a window.
                {181, "window-other"},  // Any type of window that must be visible in the image
                                        // (replaces window).
                {182, "wood"},          // Raw wood materials (incl. logs).
                {183, "other"}          // Any other type of object.
            };
        }();

        inline static std::unordered_map<std::string, int> stuff_label_to_id = {};
        inline static std::map<int, std::string> stuff_id_to_supercategory =
            []() -> std::map<int, std::string> {
            auto json = nlohmann::json::parse(CocoStuffLabels::json);
            CocoStuffLabels::stuff_label_to_id = CocoObjectLabels2017::obj_label_to_id;
            stuff_label_to_id["unlabeled"] = 0;
            stuff_label_to_id["street sign"] = 12;
            stuff_label_to_id["hat"] = 26;
            stuff_label_to_id["shoe"] = 29;
            stuff_label_to_id["eye glasses"] = 30;
            stuff_label_to_id["plate"] = 45;
            stuff_label_to_id["mirror"] = 66;
            stuff_label_to_id["window"] = 68;
            stuff_label_to_id["desk"] = 69;
            stuff_label_to_id["door"] = 71;
            stuff_label_to_id["blender"] = 83;
            stuff_label_to_id["hair brush"] = 91;
            std::map<int, std::string> stuff_id_to_supercategory =
                CocoObjectLabels2017::obj_id_to_supercategory;
            for (auto& cat: json["categories"]) {
                EXPECT_TRUE(
                    CocoStuffLabels::stuff_id_to_label[cat["id"].get<int>()] ==
                    cat["name"].get<std::string>());
                CocoStuffLabels::stuff_label_to_id[cat["name"].get<std::string>()] =
                    cat["id"].get<int>();
                stuff_id_to_supercategory[cat["id"].get<int>()] =
                    cat["supercategory"].get<std::string>();
            }
            return stuff_id_to_supercategory;
        }();
    };
}  // namespace erl::env

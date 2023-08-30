#include "erl_common/test_helper.hpp"
#include "erl_env/coco_labels.hpp"

TEST(ERL_ENV, CocoObjectLabels2017) {
    EXPECT_EQ(erl::env::CocoObjectLabels2017::obj_id_to_label.size(), 80);
    for (auto &[id, label]: erl::env::CocoObjectLabels2017::obj_id_to_label) {
        EXPECT_EQ(id, erl::env::CocoObjectLabels2017::obj_label_to_id[label]);
        std::cout << id << ": " << label << std::endl;
    }
}

TEST(ERL_ENV, CocoStuffLabels) {
    EXPECT_EQ(erl::env::CocoStuffLabels::stuff_id_to_label.size(), 184);

    for (auto &[id, label]: erl::env::CocoStuffLabels::stuff_id_to_label) {
        EXPECT_EQ(id, erl::env::CocoStuffLabels::stuff_label_to_id[label]);
        std::cout << id << ": " << label << std::endl;
    }
}

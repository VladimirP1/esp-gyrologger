#include <utility>

extern "C" {
#include <ff.h>
}

inline std::pair<int, int> get_free_space_kb() {
    constexpr int kSectorBytes = 4096;
    FATFS* fs;
    DWORD fre_clust, fre_sect, tot_sect;
    /* Get volume information and free clusters of drive 0 */
    FRESULT res = f_getfree("0:", &fre_clust, &fs);
    /* Get total sectors and free sectors */
    tot_sect = (fs->n_fatent - 2) * fs->csize;
    fre_sect = fre_clust * fs->csize;
    return {kSectorBytes * fre_sect / 1024, kSectorBytes * tot_sect / 1024};
}
/* Copyright (C) 2010-2018 (see AUTHORS file for a list of contributors)
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <volk_gnsssdr/volk_gnsssdr_prefs.h>


void volk_gnsssdr_get_config_path(char *path)
{
    if (!path) return;
    const char *suffix = "/.volk_gnsssdr/volk_gnsssdr_config";
    const char *suffix2 = "/volk_gnsssdr/volk_gnsssdr_config";  // non-hidden
    char *home = NULL;

    //allows config redirection via env variable
    home = getenv("VOLK_CONFIGPATH");
    if (home != NULL)
        {
            strncpy(path, home, 512);
            strcat(path, suffix2);
            return;
        }

    if (home == NULL) home = getenv("HOME");
    if (home == NULL) home = getenv("APPDATA");
    if (home == NULL)
        {
            path[0] = 0;
            return;
        }
    strncpy(path, home, 512);
    strcat(path, suffix);
}

size_t volk_gnsssdr_load_preferences(volk_gnsssdr_arch_pref_t **prefs_res)
{
    FILE *config_file;
    char path[512], line[512];
    size_t n_arch_prefs = 0;
    volk_gnsssdr_arch_pref_t *prefs = NULL;

    //get the config path
    volk_gnsssdr_get_config_path(path);
    if (!path[0]) return n_arch_prefs;  //no prefs found
    config_file = fopen(path, "r");
    if (!config_file) return n_arch_prefs;  //no prefs found

    //reset the file pointer and write the prefs into volk_gnsssdr_arch_prefs
    while (fgets(line, sizeof(line), config_file) != NULL)
        {
            prefs = (volk_gnsssdr_arch_pref_t *)realloc(prefs, (n_arch_prefs + 1) * sizeof(*prefs));
            volk_gnsssdr_arch_pref_t *p = prefs + n_arch_prefs;
            if (sscanf(line, "%s %s %s", p->name, p->impl_a, p->impl_u) == 3 && !strncmp(p->name, "volk_gnsssdr_", 5))
                {
                    n_arch_prefs++;
                }
        }
    fclose(config_file);
    *prefs_res = prefs;
    return n_arch_prefs;
}

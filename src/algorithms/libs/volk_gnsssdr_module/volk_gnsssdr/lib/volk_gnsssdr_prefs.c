/*
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2019 (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#if defined(_MSC_VER)
#include <io.h>
#define access _access
#define F_OK 0
#else
#include <unistd.h>
#endif
#include <volk_gnsssdr/volk_gnsssdr_prefs.h>


void volk_gnsssdr_get_config_path(char *path, bool read)
{
    if (!path) return;
    const char *suffix = "/.volk_gnsssdr/volk_gnsssdr_config";
    const char *suffix2 = "/volk_gnsssdr/volk_gnsssdr_config";  // non-hidden
    char *home = NULL;

    // allows config redirection via env variable
    home = getenv("VOLK_CONFIGPATH");
    if (home != NULL)
        {
            strncpy(path, home, 512);
            strcat(path, suffix2);
            if (!read || (access(path, F_OK) != -1))
                {
                    return;
                }
        }

    // check for user-local config file
    home = getenv("HOME");
    if (home != NULL)
        {
            strncpy(path, home, 512);
            strcat(path, suffix);
            if (!read || (access(path, F_OK) != -1))
                {
                    return;
                }
        }

    // check for config file in APPDATA (Windows)
    home = getenv("APPDATA");
    if (home != NULL)
        {
            strncpy(path, home, 512);
            strcat(path, suffix);
            if (!read || (access(path, F_OK) != -1))
                {
                    return;
                }
        }

    // check for system-wide config file
    if (access("/etc/volk_gnsssdr/volk_gnsssdr_config", F_OK) != -1)
        {
            strncpy(path, "/etc", 512);
            strcat(path, suffix2);
            return;
        }

    // if nothing exists, write to HOME or APPDATA
    home = getenv("HOME");
    if (home == NULL) home = getenv("APPDATA");
    if (home != NULL)
        {
            strncpy(path, home, 512);
            strcat(path, suffix);
            return;
        }
    path[0] = 0;
    return;
}


size_t volk_gnsssdr_load_preferences(volk_gnsssdr_arch_pref_t **prefs_res)
{
    FILE *config_file;
    char path[512], line[512];
    size_t n_arch_prefs = 0;
    volk_gnsssdr_arch_pref_t *prefs = NULL;

    // get the config path
    volk_gnsssdr_get_config_path(path, true);
    if (!path[0]) return n_arch_prefs;  //no prefs found
    config_file = fopen(path, "r");
    if (!config_file) return n_arch_prefs;  //no prefs found

    // reset the file pointer and write the prefs into volk_gnsssdr_arch_prefs
    while (fgets(line, sizeof(line), config_file) != NULL)
        {
            void *new_prefs = realloc(prefs, (n_arch_prefs + 1) * sizeof(*prefs));
            if (!new_prefs)
                {
                    printf("volk_gnsssdr_load_preferences: bad malloc\n");
                    break;
                }
            prefs = (volk_gnsssdr_arch_pref_t *)new_prefs;
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

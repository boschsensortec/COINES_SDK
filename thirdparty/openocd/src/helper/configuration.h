/***************************************************************************
 *   Copyright (C) 2004, 2005 by Dominic Rath                              *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_HELPER_CONFIGURATION_H
#define OPENOCD_HELPER_CONFIGURATION_H

#include <helper/command.h>

int parse_cmdline_args(struct command_context *cmd_ctx,
		int argc, char *argv[]);

int parse_config_file(struct command_context *cmd_ctx);
void add_config_command(const char *cfg);

void add_script_search_dir(const char *dir);

void free_config(void);

int configuration_output_handler(struct command_context *cmd_ctx,
		const char *line);

FILE *open_file_from_path(const char *file, const char *mode);

char *find_file(const char *name);
char *get_home_dir(const char *append_path);

#endif /* OPENOCD_HELPER_CONFIGURATION_H */
